"""Own one command's process group and forward terminal interruption promptly.

Linux subreaping keeps a ros2 wrapper's orphaned children under this launcher.
Only this command's new process group is signalled. Flight modes receive SIGINT
without forced termination so their existing landing routine can finish.
"""
import ctypes
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


FAST_MODES=frozenset(('doctor','mocap','capture','survey','backend','agents',
                      'check','offline','preview','observe'))
FLIGHT_MODES=frozenset(('fly','acquire'))
INTERRUPT_GRACE_S=.75
TERMINATE_GRACE_S=.50
KILL_GRACE_S=.50


def _subreaper():
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(36,1,0,0,0)!=0:
        raise OSError(ctypes.get_errno(),'Cannot establish owned-child cleanup')


def _alive(group):
    try:os.killpg(group,0);return True
    except ProcessLookupError:return False


def _reap(proc):
    proc.poll()
    while True:
        try:pid,status=os.waitpid(-proc.pid,os.WNOHANG)
        except ChildProcessError:break
        if not pid:break
        if pid==proc.pid:proc.returncode=os.waitstatus_to_exitcode(status)


def run(command,mode,workspace):
    """Run until the owned group is gone; flight recovery has no kill timer."""
    if mode not in FAST_MODES|FLIGHT_MODES:raise ValueError('Unknown shutdown mode')
    _subreaper();proc=None;requested=None;began=time.monotonic();stage=0
    flight=mode in FLIGHT_MODES;root_finished=None;interrupted_at=None
    report=dict(mode=mode,flight_recovery_protected=flight,signals=[],
                supervisor_pid=os.getpid(),scope='Only the process group created by this command')
    def send(sig):
        if proc is None:return
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:return
        report['signals'].append(dict(signal=sig.name,monotonic_s=time.monotonic(),process_group=proc.pid))
    def interrupt(signum,frame):
        nonlocal requested,interrupted_at,stage
        if requested is not None:return  # Repeated Ctrl+C must not abort landing.
        requested=signum;interrupted_at=time.monotonic();stage=1
        send(signal.SIGINT)
    old={s:signal.signal(s,interrupt) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    try:
        proc=subprocess.Popen(command,start_new_session=True)
        report.update(child_pid=proc.pid,process_group=proc.pid)
        if requested is not None:send(signal.SIGINT)
        notice=False
        while True:
            _reap(proc)
            if proc.returncode is not None and not _alive(proc.pid):break
            now=time.monotonic()
            if proc.returncode is not None and requested is None:
                root_finished=now if root_finished is None else root_finished
                if now-root_finished>=.10:
                    # A CLI wrapper must not leave its children running on exit.
                    interrupt(signal.SIGINT,None)
                    report['wrapper_exited_before_children']=True
            if requested is not None:
                if not notice:
                    message='landing/recovery will finish before exit' if flight else 'stopping this command and its child nodes'
                    print('[single_mission] Ctrl+C/shutdown: '+message,flush=True,file=sys.stderr);notice=True
                elapsed=now-interrupted_at
                if not flight:
                    if stage==1 and elapsed>=INTERRUPT_GRACE_S:send(signal.SIGTERM);stage=2
                    if stage==2 and elapsed>=INTERRUPT_GRACE_S+TERMINATE_GRACE_S:send(signal.SIGKILL);stage=3
                    if stage==3 and elapsed>=INTERRUPT_GRACE_S+TERMINATE_GRACE_S+KILL_GRACE_S:
                        _reap(proc)
                        if _alive(proc.pid):raise RuntimeError('Owned process group did not exit after SIGKILL; inspect shutdown report')
            time.sleep(.01)
        return 128+requested if requested is not None else (proc.returncode if proc.returncode>=0 else 128-proc.returncode)
    finally:
        # Ordinary exceptions still request cleanup; no force-kill flight recovery.
        if proc is not None:
            _reap(proc)
            if _alive(proc.pid) and requested is None:send(signal.SIGINT)
            report.update(child_returncode=proc.returncode,process_group_remaining=_alive(proc.pid))
        report.update(wall_s=time.monotonic()-began,interrupted=requested is not None,
            shutdown_s=None if interrupted_at is None else time.monotonic()-interrupted_at)
        for sig,handler in old.items():signal.signal(sig,handler)
        try:
            directory=Path(workspace).expanduser()/'results/single_node_shutdown';directory.mkdir(parents=True,exist_ok=True)
            name=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_'+uuid.uuid4().hex[:6]+'.json'
            (directory/name).write_text(json.dumps(report,indent=2))
        except OSError as exc:print('[single_mission] Could not write shutdown report: '+str(exc),file=sys.stderr)
