"""Bounded file I/O workers, kept off ROS callbacks and the GUI thread."""
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import threading
import time


class CommandTail:
    def __init__(self, root, explicit='', birth=None):
        self.root, self.explicit = Path(root).expanduser(), explicit
        self.birth = time.time() if birth is None else birth
        self.path, self.handle, self.header, self.inode = None, None, None, None
        self.queue = queue.Queue(maxsize=4096)
        self.dropped, self.errors, self.stop = 0, 0, threading.Event()
        self.worker = threading.Thread(target=self.run, daemon=True, name='live-command-csv')
        self.worker.start()

    def choose(self):
        if self.explicit:
            p = Path(self.explicit).expanduser()
            return p if p.is_file() else None
        candidates = [p for p in self.root.glob('single_mission_*/position_commands.csv')
                      if p.stat().st_mtime >= self.birth]
        return max(candidates, key=lambda p: p.stat().st_mtime, default=None)

    def poll(self, path):
        info = path.stat(); ident = (info.st_dev, info.st_ino)
        if path != self.path or ident != self.inode or (self.handle and info.st_size < self.handle.tell()):
            if self.handle: self.handle.close()
            self.handle = path.open('r', newline='')
            self.path, self.inode, self.header = path, ident, None
        for _ in range(600):  # Bound work per poll, including a large pre-existing file.
            position = self.handle.tell(); line = self.handle.readline(16384)
            if not line: break
            if not line.endswith('\n'):
                self.handle.seek(position); break  # Retry a partially written CSV record.
            values = next(csv.reader([line]))
            if self.header is None:
                if 'published_z_m' not in values or 'ros_ns' not in values:
                    raise ValueError('Expected mission position_commands.csv schema')
                self.header = values; continue
            if len(values) != len(self.header): self.errors += 1; continue
            try: self.queue.put_nowait((str(path), dict(zip(self.header, values))))
            except queue.Full: self.dropped += 1

    def run(self):
        try:
            while not self.stop.is_set():
                try:
                    p = self.choose()
                    if p is not None: self.poll(p)
                except (OSError, ValueError, csv.Error): self.errors += 1
                self.stop.wait(.1)
        finally:
            if self.handle: self.handle.close()

    def close(self):
        self.stop.set(); self.worker.join(timeout=2.)


class EventWriter:
    def __init__(self, output, cfg):
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.directory = Path(output).expanduser()/stamp
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory/'configuration.json').write_text(json.dumps(cfg, indent=2, allow_nan=False))
        self.queue = queue.Queue(maxsize=4096)
        self.dropped, self.errors, self.written = 0, [], 0
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self.run, daemon=True, name='live-plot-recorder')
        self.worker.start()

    def submit(self, group, event):
        try: self.queue.put_nowait((group, event))
        except queue.Full: self.dropped += 1

    def run(self):
        files = {}; last_flush = time.monotonic()
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    group, event = self.queue.get(timeout=.1)
                except queue.Empty: continue
                try:
                    if group not in files: files[group] = (self.directory/(group+'.jsonl')).open('a')
                    files[group].write(json.dumps(event, allow_nan=False, separators=(',', ':'))+'\n')
                    self.written += 1
                    if time.monotonic()-last_flush >= 1.:
                        for f in files.values(): f.flush()
                        last_flush = time.monotonic()
                except (OSError, ValueError, TypeError) as exc:
                    if len(self.errors) < 10: self.errors.append(str(exc))
                finally: self.queue.task_done()
        finally:
            for f in files.values(): f.close()

    def close(self):
        self.stop.set(); self.worker.join(timeout=3.)
        summary = dict(written_events=self.written, dropped_events=self.dropped, errors=self.errors,
                       queue_remaining=self.queue.qsize(), worker_still_running=self.worker.is_alive())
        (self.directory/'summary.json').write_text(json.dumps(summary, indent=2))
        return summary
