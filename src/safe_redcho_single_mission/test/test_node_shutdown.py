"""Harmless subprocesses only: terminal Ctrl+C, descendants and landing time."""
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import time
import fcntl
import termios
import pytest
from safe_redcho_single_mission.node_shutdown import FAST_MODES,FLIGHT_MODES
from safe_redcho_single_mission.run import MODES


def wait_file(path,proc,timeout=5.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if path.exists():return
        assert proc.poll() is None,'Test process exited before its ready marker'
        time.sleep(.01)
    raise AssertionError('No ready marker')


def start(tmp_path,code,mode='mocap',terminal=False):
    child=tmp_path/'child.py';child.write_text(code)
    driver=tmp_path/'driver.py'
    driver.write_text('import json,sys\nfrom safe_redcho_single_mission.node_shutdown import run\n'
        'raise SystemExit(run(json.loads(sys.argv[3]),sys.argv[1],sys.argv[2]))\n')
    command=[sys.executable,'-s',str(driver),mode,str(tmp_path),json.dumps([sys.executable,'-s',str(child)])]
    if terminal:
        master,slave=pty.openpty()
        def setup():
            os.setsid();fcntl.ioctl(slave,termios.TIOCSCTTY,0);os.tcsetpgrp(slave,os.getpgrp())
        proc=subprocess.Popen(command,stdin=slave,stdout=slave,stderr=slave,preexec_fn=setup)
        os.close(slave);return proc,master
    log=(tmp_path/'console.log').open('w')
    proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT);log.close();return proc,None


def report(tmp_path):
    paths=list((tmp_path/'results/single_node_shutdown').glob('*.json'));assert len(paths)==1
    result=json.loads(paths[0].read_text());assert not result['process_group_remaining'];return result


def cleanup_test(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=5.)
        except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=2.)


def test_every_cli_mode_has_an_explicit_shutdown_policy():
    assert set(MODES)==FAST_MODES|FLIGHT_MODES


def test_successful_command_exit_is_preserved(tmp_path):
    proc,_=start(tmp_path,'pass\n')
    try:
        assert proc.wait(timeout=5.)==0
        assert report(tmp_path)['signals']==[]
    finally:cleanup_test(proc)


@pytest.mark.parametrize('sig',[signal.SIGINT,signal.SIGTERM,signal.SIGHUP])
def test_hung_child_is_force_stopped_within_short_deadline(tmp_path,sig):
    ready=tmp_path/'ready'
    code='import signal,time,pathlib\nfor s in (signal.SIGINT,signal.SIGTERM):signal.signal(s,signal.SIG_IGN)\n'+f'pathlib.Path({str(ready)!r}).touch()\ntime.sleep(30)\n'
    proc,_=start(tmp_path,code)
    try:
        wait_file(ready,proc);began=time.monotonic();proc.send_signal(sig)
        assert proc.wait(timeout=4.)==128+sig
        assert time.monotonic()-began<3.
        r=report(tmp_path);assert [x['signal'] for x in r['signals']]==['SIGINT','SIGTERM','SIGKILL']
    finally:cleanup_test(proc)


def test_actual_terminal_ctrl_c_stops_command(tmp_path):
    ready=tmp_path/'ready'
    proc,master=start(tmp_path,f'import pathlib,time\npathlib.Path({str(ready)!r}).touch()\ntime.sleep(30)\n',terminal=True)
    try:
        wait_file(ready,proc);os.write(master,b'\x03')
        assert proc.wait(timeout=4.)==130
        r=report(tmp_path);assert r['interrupted'] and r['shutdown_s']<1.
    finally:cleanup_test(proc);os.close(master)


def test_orphaned_grandchild_is_reaped_and_other_process_is_untouched(tmp_path):
    ready=tmp_path/'ready';leaf=tmp_path/'leaf.py'
    leaf.write_text('import signal,time,os,pathlib\nfor s in (signal.SIGINT,signal.SIGTERM):signal.signal(s,signal.SIG_IGN)\n'+f'pathlib.Path({str(ready)!r}).write_text(str(os.getpid()))\ntime.sleep(30)\n')
    code='import signal,subprocess,sys,time\nfor s in (signal.SIGINT,signal.SIGTERM):signal.signal(s,signal.SIG_IGN)\n'+f'subprocess.Popen([sys.executable,"-s",{str(leaf)!r}])\ntime.sleep(30)\n'
    other=subprocess.Popen([sys.executable,'-s','-c','import time;time.sleep(30)'])
    proc,_=start(tmp_path,code,mode='backend')
    try:
        wait_file(ready,proc);leaf_pid=int(ready.read_text());proc.send_signal(signal.SIGINT)
        assert proc.wait(timeout=4.)==130
        assert not Path('/proc',str(leaf_pid)).exists()
        assert other.poll() is None
        report(tmp_path)
    finally:cleanup_test(proc);other.terminate();other.wait(timeout=2.)


def test_normal_wrapper_exit_does_not_leave_its_child(tmp_path):
    ready=tmp_path/'ready';leaf=tmp_path/'leaf.py'
    leaf.write_text('import pathlib,time\n'+f'pathlib.Path({str(ready)!r}).touch()\ntime.sleep(30)\n')
    code='import subprocess,sys,pathlib,time\n'+f'subprocess.Popen([sys.executable,"-s",{str(leaf)!r}])\n'+f'while not pathlib.Path({str(ready)!r}).exists():time.sleep(.01)\n'
    proc,_=start(tmp_path,code)
    try:
        assert proc.wait(timeout=5.)==130
        assert report(tmp_path)['wrapper_exited_before_children']
    finally:cleanup_test(proc)


@pytest.mark.parametrize('mode',['fly','acquire'])
def test_flight_wrapper_exit_waits_for_recovery_despite_repeated_ctrl_c(tmp_path,mode):
    ready=tmp_path/'ready';done=tmp_path/'landed';leaf=tmp_path/'leaf.py'
    leaf.write_text('import signal,time,pathlib\n'
        'def stop(*args):\n'
        ' signal.signal(signal.SIGINT,signal.SIG_IGN)\n'
        ' signal.signal(signal.SIGTERM,signal.SIG_IGN)\n'
        ' time.sleep(1.8)\n'+f' pathlib.Path({str(done)!r}).touch()\n'
        ' raise SystemExit(0)\n'
        'signal.signal(signal.SIGINT,stop)\n'+f'pathlib.Path({str(ready)!r}).touch()\ntime.sleep(30)\n')
    code='import subprocess,sys,time\n'+f'subprocess.Popen([sys.executable,"-s",{str(leaf)!r}])\ntime.sleep(30)\n'
    proc,_=start(tmp_path,code,mode=mode)
    try:
        wait_file(ready,proc);proc.send_signal(signal.SIGINT);time.sleep(.1);proc.send_signal(signal.SIGINT)
        assert proc.wait(timeout=5.)==130
        assert done.exists()
        r=report(tmp_path);assert r['flight_recovery_protected']
        assert [x['signal'] for x in r['signals']]==['SIGINT']
        assert r['shutdown_s']>=1.8
    finally:cleanup_test(proc)
