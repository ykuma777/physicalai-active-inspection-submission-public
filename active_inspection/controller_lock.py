"""Single motion-executor lock shared by the trial shell and controller."""
from __future__ import annotations
import fcntl
import os
from pathlib import Path

LOCK_PATH=Path('/tmp/physicalai-active-inspection-trial.lock')

def acquire():
    inherited=os.environ.get('ACTIVE_INSPECTION_LOCK_FD')
    if inherited is not None:
        fd=int(inherited)
        expected=LOCK_PATH.stat();actual=os.fstat(fd)
        if (expected.st_dev,expected.st_ino)!=(actual.st_dev,actual.st_ino):
            raise RuntimeError('invalid inherited motion-executor lock; no command sent')
        # The shell owns this open file description through the whole trial.
        # Reasserting its flock here confirms that the controller inherited it.
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        return fd,False
    fd=os.open(LOCK_PATH,os.O_CREAT|os.O_RDWR,0o600)
    try:
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(fd)
        raise RuntimeError('another motion executor owns the controller; no command sent') from exc
    return fd,True

def release(fd,owned):
    if fd is not None and owned:
        fcntl.flock(fd,fcntl.LOCK_UN)
        os.close(fd)
