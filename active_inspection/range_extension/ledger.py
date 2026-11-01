"""Append-only trial consumption ledger; crashes never make a slot reusable."""
from __future__ import annotations
from pathlib import Path
from contextlib import AbstractContextManager
import json,os
from common import write,read,append,now

TERMINAL={'PASS','QUALITY_FAIL','PROTOCOL_STOP','FAULT_STOP','INTERRUPTED'}

class FileLease(AbstractContextManager):
    def __init__(self,path):self.path=Path(path);self.file=None;self.defer_unlock=False
    def __enter__(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.file=self.path.open('a+b');self.file.seek(0)
        if self.path.stat().st_size==0:self.file.write(b'0');self.file.flush()
        self.file.seek(0)
        try:
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(self.file.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(self.file.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as e:
            self.file.close();self.file=None
            raise RuntimeError('another runner owns the lease; nothing started') from e
        return self
    def __exit__(self,*args):
        if self.file:
            self.file.seek(0)
            if self.defer_unlock:
                # A still-running child inherited the same flock description.
                # Close only this parent's copy, leaving that child's lease held.
                self.file.close();self.file=None;return
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(self.file.fileno(),msvcrt.LK_UNLCK,1)
            else:
                import fcntl
                fcntl.flock(self.file.fileno(),fcntl.LOCK_UN)
            self.file.close();self.file=None

class Ledger:
    """Caller must hold FileLease for all mutations."""
    def __init__(self,root,plan,plan_sha256):
        self.root=Path(root);self.plan=plan;self.plan_sha256=plan_sha256
        self.events=self.root/'progress_events.jsonl';self.snapshot=self.root/'progress_ledger.json'
        self.states={};self.trials={t['run_id']:t for t in plan['trials']}
        if self.events.exists():
            for line in self.events.read_text(encoding='utf-8').splitlines():
                e=json.loads(line) # Never silently discard a partial/corrupt commit.
                if e['plan_sha256']!=plan_sha256:raise RuntimeError('plan changed after reservation')
                rid=e['run_id']
                if rid not in self.trials:raise RuntimeError('unplanned trial in ledger')
                if e['phase']!=self.trials[rid]['phase'] or e['status'] not in TERMINAL|{'RESERVED'}:raise RuntimeError('invalid ledger phase/status')
                if e['status']=='RESERVED' and rid in self.states:raise RuntimeError('duplicate reservation')
                if self.states.get(rid,{}).get('status') in TERMINAL:raise RuntimeError('terminal trial overwritten')
                if e['status'] in TERMINAL and self.states.get(rid,{}).get('status')!='RESERVED':raise RuntimeError('terminal entry without reservation')
                self.states[rid]=e
        if len(self.states)>28:raise RuntimeError('total consumption exceeded')
    def emit(self,run_id,status,**kw):
        row={'run_id':run_id,'status':status,'phase':self.trials[run_id]['phase'],
             'plan_sha256':self.plan_sha256,'at_utc':now(),**kw}
        append(self.events,row);self.states[run_id]=row
        write(self.snapshot,{'plan_sha256':self.plan_sha256,'updated_at_utc':now(),
              'consumed_trials':len(self.states),'states':self.states,
              'main_trials_consumed':sum(e['phase']!='path_probe' for e in self.states.values()),
              'path_probes_consumed':sum(e['phase']=='path_probe' for e in self.states.values())})
        return row
    def recover_interrupted(self):
        recovered=[]
        for rid,t in self.trials.items():
            intent=self.root/'runs'/rid/'reservation.json'
            if intent.parent.exists() and not intent.exists() and rid not in self.states:
                self.emit(rid,'RESERVED',orphan_directory_without_intent=True)
            if intent.exists() and rid not in self.states:
                row=read(intent)
                if row['plan_sha256']!=self.plan_sha256:raise RuntimeError('orphan belongs to changed plan')
                self.emit(rid,'RESERVED',orphan_reservation=True)
            if self.states.get(rid,{}).get('status')=='RESERVED':
                self.emit(rid,'INTERRUPTED',reason='previous invocation ended before terminal commit; no automatic replay')
                recovered.append(rid)
        return recovered
    def reserve(self,t):
        rid=t['run_id']
        if rid not in self.trials or t!=self.trials[rid]:raise RuntimeError('reservation differs from frozen canonical trial')
        if rid in self.states:raise RuntimeError('completed/failed/reserved trial cannot be replayed: '+rid)
        main=sum(e['phase']!='path_probe' for e in self.states.values())
        probes=sum(e['phase']=='path_probe' for e in self.states.values())
        if len(self.states)>=28 or (t['phase']=='path_probe' and probes>=2) or (t['phase']!='path_probe' and main>=26):raise RuntimeError('consumed trial budget exhausted')
        rd=self.root/'runs'/rid;rd.mkdir(parents=True,exist_ok=False)
        write(rd/'reservation.json',{'run_id':rid,'plan_sha256':self.plan_sha256,'trial':t,'at_utc':now()})
        self.emit(rid,'RESERVED',run_directory=str(rd))
        return rd
    def finish(self,rid,status,**kw):
        if status not in TERMINAL or self.states.get(rid,{}).get('status')!='RESERVED':raise RuntimeError('invalid terminal transition')
        return self.emit(rid,status,**kw)
    def pending(self,phase):return[t for t in self.plan['trials'] if t['phase']==phase and t['run_id'] not in self.states]
    def blocking(self):return[e for e in self.states.values() if e['status'] in ['FAULT_STOP','PROTOCOL_STOP','INTERRUPTED']]
