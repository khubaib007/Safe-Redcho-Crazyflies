"""ROS-independent CSV, selection, and timing utilities."""
import csv
from datetime import datetime, timezone
from fnmatch import fnmatchcase
import json
import math
from pathlib import Path
import queue
import threading
import time

FIELDS = ['record_utc', 'receipt_monotonic_ns', 'receipt_ros_ns', 'source_stamp_ns',
          'firmware_timestamp_raw', 'session', 'seq', 'agent', 'stage', 'source_topic',
          'source', 'frame', 'variable', 'value', 'unit', 'valid', 'reason']


def flatten(value, prefix=''):
    if isinstance(value, dict):
        for key, item in value.items():
            yield from flatten(item, f'{prefix}.{key}' if prefix else str(key))
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value): yield from flatten(item, f'{prefix}.{index}')
    else:
        yield prefix, value


def selected(path, switches, includes=('*',), excludes=()):
    if not any(fnmatchcase(path, pattern) for pattern in includes): return False
    if any(fnmatchcase(path, pattern) for pattern in excludes): return False
    # A disabled parent always disables its descendants.
    matches = [(len(pattern.replace('*', '')), flag) for pattern, flag in switches.items()
               if fnmatchcase(path, pattern) or path.startswith(pattern+'.') or path == pattern]
    return all(flag for _,flag in matches)


def unit_for(path):
    if path.startswith('published_full_state.'):
        if '.pose.position.' in path: return 'm'
        if '.twist.linear.' in path: return 'm/s'
        if '.twist.angular.' in path: return 'rad/s'
        if '.acc.' in path: return 'm/s^2'
        if '.pose.orientation.' in path: return '1'
    for suffix, unit in [('mps4', 'm/s^4'), ('mps3', 'm/s^3'), ('mps2', 'm/s^2'), ('mps', 'm/s'),
                         ('_rad', 'rad'), ('_deg', 'deg'), ('_g', 'g'), ('voltage_v', 'V'),
                         ('_wall_s', 's'), ('_age_s', 's'), ('_m', 'm')]:
        if any(part.endswith(suffix) for part in path.split('.')): return unit
    return ''


def scalar_text(value):
    if value is None: return '', False
    if isinstance(value, float) and not math.isfinite(value): return '', False
    if isinstance(value, (dict, list, tuple)): return json.dumps(value, allow_nan=False), True
    return value, True


def stamp_ns(message):
    header = getattr(message, 'header', None)
    if header is None: return ''
    stamp = header.stamp
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def vector(value):
    return {'x': float(value.x), 'y': float(value.y), 'z': float(value.z)}


def pose_data(pose):
    q = pose.orientation
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-9: raise ValueError('Invalid quaternion')
    x, y, z, w = (v/norm for v in (q.x, q.y, q.z, q.w))
    return {'position_m': vector(pose.position),
            'orientation_xyzw': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w},
            'rpy_rad': {'roll': math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                        'pitch': math.asin(max(-1., min(1., 2*(w*y-z*x)))),
                        'yaw': math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))}}


class AsyncCSV:
    """Bounded nonblocking enqueue; only the worker performs disk I/O."""
    def __init__(self, directory, max_batches=2000, flush_s=1., layout='wide'):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        if layout not in ('wide','long'): raise ValueError('CSV layout must be wide or long')
        self.queue, self.flush_s, self.layout = queue.Queue(maxsize=max_batches), flush_s, layout
        self.dropped_batches, self.errors, self.written_rows = 0, [], 0
        self.closed, self._stop = False, threading.Event()
        self.worker = threading.Thread(target=self._run, daemon=True, name='telemetry_csv_writer')
        self.worker.start()

    def submit(self, stream, rows):
        if self.closed or not rows: return
        try: self.queue.put_nowait((stream, rows))
        except queue.Full: self.dropped_batches += 1

    def _run(self):
        handles, writers, columns, parts, dictionary, last_flush = {}, {}, {}, {}, {}, time.monotonic()
        metadata_fields = [k for k in FIELDS if k not in ('variable','value','unit','valid','reason')]
        try:
            while not self._stop.is_set() or not self.queue.empty():
                try: item = self.queue.get(timeout=.1)
                except queue.Empty:
                    item = None
                if item is not None:
                    stream, rows = item
                    try:
                        if self.layout == 'wide':
                            names = {r['variable'] for r in rows}
                            missing = names-set(columns.get(stream,[]))
                            if stream not in writers or missing:
                                if stream in handles: handles[stream].close()
                                columns[stream] = sorted(set(columns.get(stream,[]))|names)
                                parts[stream] = parts.get(stream,0)+1
                                suffix = '' if parts[stream] == 1 else f'_part{parts[stream]:04d}'
                                handle = (self.directory/f'{stream}{suffix}.csv').open('x', newline='')
                                handles[stream] = handle
                                writers[stream] = csv.DictWriter(handle,fieldnames=metadata_fields+['invalid_fields']+columns[stream])
                                writers[stream].writeheader()
                            record = {k:rows[0][k] for k in metadata_fields}
                            record.update({r['variable']:r['value'] for r in rows})
                            record['invalid_fields'] = ';'.join(r['variable']+':'+r['reason'] for r in rows if not r['valid'])
                            for r in rows: dictionary[r['variable']] = {'unit':r['unit']}
                            writers[stream].writerow(record)
                            self.written_rows += 1
                        else:
                            if stream not in writers:
                                handle = (self.directory/f'{stream}.csv').open('x', newline='')
                                handles[stream], writers[stream] = handle, csv.DictWriter(handle, fieldnames=FIELDS)
                                writers[stream].writeheader()
                            writers[stream].writerows(rows)
                            self.written_rows += len(rows)
                    except Exception as exc:
                        if len(self.errors) < 20: self.errors.append(str(exc))
                    finally: self.queue.task_done()
                if time.monotonic()-last_flush >= self.flush_s:
                    for handle in handles.values():
                        try: handle.flush()
                        except Exception as exc:
                            if len(self.errors) < 20: self.errors.append(str(exc))
                    last_flush = time.monotonic()
        finally:
            for handle in handles.values():
                try: handle.close()
                except Exception as exc: self.errors.append(str(exc))
            (self.directory/'column_dictionary.json').write_text(json.dumps(dictionary,indent=2)+'\n')

    def close(self):
        self.closed = True; self._stop.set(); self.worker.join(timeout=10.)
        summary = {'written_rows': self.written_rows, 'dropped_batches': self.dropped_batches,
                   'write_errors': self.errors, 'queue_remaining': self.queue.qsize(),
                   'writer_still_running': self.worker.is_alive()}
        (self.directory/'writer_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        return summary


def make_rows(data, metadata, switches, includes=('*',), excludes=()):
    rows = []
    for path, value in flatten(data):
        if not selected(path, switches, includes, excludes): continue
        text, valid = scalar_text(value)
        rows.append(dict(metadata, variable=path, value=text, valid=int(valid),
                         unit=unit_for(path), reason='' if valid else 'missing_or_nonfinite'))
    return rows


def utc_now(): return datetime.now(timezone.utc).isoformat()
