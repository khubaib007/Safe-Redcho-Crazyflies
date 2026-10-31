"""Startup custom logging only; never connects to a radio from a recorder."""
from copy import deepcopy

def enabled_blocks(profile):
    if not isinstance(profile, dict) or not profile.get('enabled', False): return {}
    output = {}
    for topic, block in profile.get('blocks', {}).items():
        if not block.get('enabled', False): continue
        variables = block['vars']
        if not variables or len(variables) > 6 or len(set(variables)) != len(variables):
            raise ValueError(f'{topic}: require 1..6 unique float log variables (<=24 payload bytes)')
        hz = int(block['frequency'])
        if hz not in (1, 2, 5, 10, 20, 25, 50): raise ValueError(f'{topic}: unsupported log frequency')
        if len(block['fields']) != len(variables): raise ValueError(f'{topic}: field/variable mapping mismatch')
        output[topic] = {'frequency': hz, 'vars': list(variables)}
    return output


def apply_logging_profile(robots, profile):
    blocks = enabled_blocks(profile)
    if not blocks: return
    # Apply at the highest per-robot priority, preserving existing logs/settings.
    for name, robot in robots['robots'].items():
        if not robot.get('enabled', True): continue
        logging = robot.setdefault('firmware_logging', {})
        logging['enabled'] = True
        custom = logging.setdefault('custom_topics', {})
        for topic, block in blocks.items():
            if topic in custom and custom[topic] != block:
                raise ValueError(f'{name}: existing incompatible custom log block {topic}')
            custom[topic] = deepcopy(block)
