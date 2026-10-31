# Release preparation: locate dependencies from the checkout root.
"""Conservative source-pattern check for global mocap broadcasts on both radios.

This does not prove compiled-binary behaviour, RF delivery or fresh peer tables.
Unrecognized code is rejected instead of assuming cross-channel coverage.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
from .fleet_configuration import load_fleet


# Exact callback body inspected in the user's 2026-10-21 source excerpt.
# This intentionally recognizes a narrow source shape, not arbitrary C++.
# Removing comments/formatting is allowed; changed statements fail closed.
INSPECTED_SPLIT_CALLBACK_BODY = r'''
    mocap_data_received_timepoints_.emplace_back(std::chrono::steady_clock::now());
    std::vector<CrazyflieBroadcaster::externalPosition> data_position;
    std::vector<CrazyflieBroadcaster::externalPose> data_pose;
    for (const auto& pose : msg->poses) {
      const auto iter = name_to_id_.find(pose.name);
      if (iter != name_to_id_.end()) {
        uint8_t id = iter->second;
        if (isnan(pose.pose.orientation.w)) {
          data_position.push_back({id,
            (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z});
        } else {
          data_pose.push_back({id,
            (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z,
            (float)pose.pose.orientation.x, (float)pose.pose.orientation.y, (float)pose.pose.orientation.z, (float)pose.pose.orientation.w});
        }
      }
    }
    if (data_position.size() > 0) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendExternalPositions(data_position);
      }
    }
    if (data_pose.size() > 0) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendExternalPoses(data_pose);
      }
    }
'''


def cpp_tokens(source):
    """Keep token boundaries and string contents while ignoring whitespace."""
    return re.findall(
        r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|'
        r'[A-Za-z_][A-Za-z_0-9]*|[0-9]+|'
        r'::|->|!=|==|>=|<=|&&|\|\||\+\+|--|[^\s]', source)


def uncomment(source):
    pattern = r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*[\s\S]*?\*/'
    return re.sub(pattern, lambda m: ' ' if m[0].startswith('/') else m[0], source)


def brace_end(text, begin):
    depth = 0
    for i in range(begin, len(text)):
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                return i
    raise ValueError('Unbalanced C++ braces in candidate callback')


def loops(body):
    found = []
    for match in re.finditer(r'for\s*\(([^()]*)\)\s*', body):
        begin = match.end()
        if begin < len(body) and body[begin] == '{':
            end = brace_end(body, begin)
            block = body[begin+1:end]
        else:
            end = body.find(';', begin)
            if end < 0:
                continue
            block = body[begin:end+1]
        found.append((match.start(), match[1], block))
    return found


def analyze_source(source):
    clean = uncomment(source)
    candidates = []
    signature = r'void\s+[\w:]+\s*\([^{};]*NamedPoseArray[^{};]*\)\s*\{'
    for match in re.finditer(signature, clean):
        end = brace_end(clean, match.end()-1)
        candidates.append(clean[match.end():end])
    reasons = []
    for body in candidates:
        if cpp_tokens(body) == cpp_tokens(INSPECTED_SPLIT_CALLBACK_BODY):
            return {
                'status': 'PASS_SOURCE_PATTERN',
                'pattern': 'inspected_split_cpp_backend_2026_10_21',
                'evidence': [
                    'Recognized names are collected globally before broadcasting; no radio/channel filter',
                    'Every broadcaster_ entry receives the complete nonempty data_pose vector',
                    'Every broadcaster_ entry receives the complete nonempty data_position vector',
                ],
                'scope': 'Exact inspected callback shape; not a compiled binary, radio or onboard peer-table test',
            }
        if re.search(r'\bbroadcaster_\b', body):
            reasons.append('Split-backend callback differs from the inspected routing statements')
            continue
        vectors = re.findall(r'std::vector\s*<\s*[\w:]*externalPose\s*>\s+(\w+)\s*;', body)
        for vector in vectors:
            declaration = re.search(r'\b'+re.escape(vector)+r'\s*;', body).start()
            if body[:declaration].count('{') != body[:declaration].count('}'):
                reasons.append('Pose vector is declared inside a nested scope')
                continue
            pose_loops = [p for p in loops(body) if re.search(r'->\s*poses\b', p[1])]
            filling = [p for p in pose_loops if re.search(r'\b'+re.escape(vector)+r'\.push_back\s*\(', p[2])]
            if not filling or any(re.search(r'\b(?:if|switch)\s*\([^)]*(?:channel|radio|group|uri)', p[2], re.I) for p in filling):
                reasons.append('No unfiltered global NamedPoseArray collection recognized')
                continue
            for position, header, block in loops(body):
                if 'm_cfBroadcasters' not in header or position <= max(p[0] for p in filling):
                    continue
                if not re.search(r'(?:\.|->)sendExternalPoses\s*\(\s*'+re.escape(vector)+r'\s*\)', block):
                    continue
                conditions = re.findall(r'\bif\s*\(([^)]*)\)', block)
                if conditions and any(vector not in expr or '.empty' not in expr for expr in conditions):
                    reasons.append('Conditional broadcaster selection not recognized')
                    continue
                return {'status': 'PASS_SOURCE_PATTERN',
                        'evidence': f'Global {vector} collection precedes all m_cfBroadcasters sendExternalPoses calls',
                        'scope': 'Source pattern only; not a compiled binary, radio or onboard peer-table test'}
    return {'status': 'UNVERIFIED', 'candidate_callbacks': len(candidates),
            'reasons': reasons or ['Expected global collection/all-broadcaster loop pattern not recognized'],
            'scope': 'Unknown source layout; cross-channel coverage must not be assumed'}


def audit_workspace(config, require_pass=True):
    profile_path=Path(config['_path'])
    root = profile_path.parents[2] if profile_path.parent.name == 'local' and profile_path.parent.parent.name == 'config' else profile_path.parent.parent
    candidates = [
        root/'src/crazyswarm2/crazyflie_server_cpp/src/crazyflie_server.cpp',
        root/'src/crazyswarm2/crazyflie/src/crazyflie_server.cpp',
    ]
    existing = [path for path in candidates if path.is_file()]
    source = existing[0] if len(existing) == 1 else None
    if source is not None:
        text = source.read_text()
        try:
            report = analyze_source(text)
        except ValueError as exc:
            report = {'status': 'UNVERIFIED', 'reasons': [str(exc)]}
        report['source_sha256'] = hashlib.sha256(text.encode()).hexdigest()
    elif not existing:
        report = {'status': 'UNVERIFIED', 'reasons': ['Crazyswarm2 source file not found']}
    else:
        report = {'status': 'UNVERIFIED', 'reasons': [
            'Both old and split backend source files exist; resolve the active checkout before auditing']}
    report['source_path'] = str(source) if source is not None else None
    report['source_candidates'] = [str(path) for path in candidates]
    out = root/'results/backend_route_audit.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, indent=2)+'\n')
    if require_pass and report['status'] != 'PASS_SOURCE_PATTERN':
        raise RuntimeError(f'Cross-radio routing unverified; inspect {out} and the poses callback in {source}. No flight commands sent.')
    return report


def main():
    parser = argparse.ArgumentParser(description='Inspect local Crazyswarm2 mocap routing; no radio commands')
    parser.add_argument('--config')
    args = parser.parse_args()
    report = audit_workspace(load_fleet(args.config), require_pass=False)
    print(json.dumps(report, indent=2))
    if report['status'] != 'PASS_SOURCE_PATTERN':
        raise SystemExit(1)
