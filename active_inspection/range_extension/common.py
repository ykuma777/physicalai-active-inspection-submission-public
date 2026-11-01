from __future__ import annotations
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import math
import os
import time
import uuid

ROOT=Path(__file__).resolve().parent
ACTIVE=ROOT.parent
REPO=ACTIVE.parent
PLAN=ROOT/'config/experiment_plan.json'

def now():return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00','Z')

def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def read(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    with temp.open('w',encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    attempts=atomic_replace(temp,path)
    if attempts>1:append(path.parent/'file_io_events.jsonl',{'at_utc':now(),'operation':'atomic_metadata_replace','target':str(path),'attempts':attempts,'motion_or_service_retries':0})

def atomic_replace(source,target,windows_retry=None):
    """Retry only Windows metadata sharing violations, never an external command."""
    enabled=os.name=='nt' if windows_retry is None else windows_retry
    for attempt in range(1,9 if enabled else 2):
        try:
            os.replace(source,target);return attempt
        except PermissionError as e:
            if not enabled or getattr(e,'winerror',None) not in (32,33) or attempt==8:raise
            time.sleep(.05) # <=0.35s; the serialized value and temp file are unchanged.

def append(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('a',encoding='utf-8') as f:
        f.write(json.dumps(value,ensure_ascii=False,allow_nan=False)+'\n');f.flush();os.fsync(f.fileno())

def finite_vector(values,length):
    if len(values)!=length or any(isinstance(v,bool) or not math.isfinite(float(v)) for v in values):raise ValueError('nonfinite/invalid vector')
    return list(map(float,values))

def assert_environment():
    expected={'ROS_DOMAIN_ID':'47','IGN_PARTITION':'physicalai_active_47','GZ_PARTITION':'physicalai_active_47'}
    actual={k:os.environ.get(k) for k in expected}
    if actual!=expected:raise RuntimeError(f'environment mismatch before ROS initialization: {actual}')
    return actual

def validate_plan(plan):
    if plan.get('schema_version')!=1:raise ValueError('unsupported plan schema')
    if plan['limits']!={'main_trials':26,'path_probe_trials':2,'total_trials':28,'max_alignment_moves':4,'max_total_moves':8,'max_wall_sec':300,'joint1_step_limit_rad':0.055}:raise ValueError('fixed trial/safety limits changed')
    trials=plan['trials'];ids=[t['run_id'] for t in trials]
    if len(trials)!=28 or len(set(ids))!=28:raise ValueError('trial IDs/count invalid')
    expected={'path_probe':2,'training':12,'tuning':2,'evaluation':12}
    for phase,count in expected.items():
        rows=[t for t in trials if t['phase']==phase]
        if len(rows)!=count:raise ValueError('phase count changed '+phase)
    places=plan['placements'];splits={s:set() for s in ['training','tuning','evaluation']}
    coords={}
    for pid,p in places.items():
        xyz=finite_vector(p['world_bottom_xyz'],3)
        if not(.13<=xyz[0]<=.26 and abs(xyz[1])<=.04 and xyz[2]==1.015):raise ValueError('placement outside retained workspace')
        if p['split'] not in splits:raise ValueError('invalid split')
        splits[p['split']].add(pid)
        key=tuple(round(x,8) for x in xyz)
        if key in coords:raise ValueError('duplicate placement coordinates')
        coords[key]=pid
        if p['needle_label']!='abnormal' or p['needle_angle_rad']!=.875 or p['yaw_rad']!=0:raise ValueError('board conditions changed')
    if [len(splits[s]) for s in ['training','tuning','evaluation']]!=[6,2,4]:raise ValueError('placement split sizes')
    for t in trials:
        p=places[t['placement_id']]
        if t['phase']!='path_probe' and p['split']!=t['phase']:raise ValueError('split leakage')
        if t['phase']=='path_probe' and p['split']!='training':raise ValueError('probe must use training placement')
        if t['phase'] in ['path_probe','training','tuning'] and t['mode']!='rule':raise ValueError('teacher collection mode changed')
        if t['mode'] not in ['fixed','rule','clone']:raise ValueError('invalid mode')
    for pid in splits['evaluation']:
        modes=[t['mode'] for t in trials if t['placement_id']==pid]
        if sorted(modes)!=['clone','fixed','rule']:raise ValueError('evaluation pairing not exact')
    for phase,key in [('training','training_run_ids'),('tuning','tuning_run_ids'),('evaluation','evaluation_run_ids')]:
        if plan[key]!=[t['run_id'] for t in trials if t['phase']==phase]:raise ValueError('run allowlist mismatch')
    if digest(ACTIVE/'configs/control.yaml')!=plan['retained_control_sha256']:raise ValueError('original capture/safety settings changed')
    return plan
