"""Explicit phase runner for the newly approved 12+2+6 campaign. No default motion."""
from pathlib import Path
import argparse,copy,json,os,sys,time,subprocess
from common import ROOT,ACTIVE,read,write,append,digest,now,assert_environment
from ledger import FileLease
from runner import command,source_environment,stop_media,OwnedChildStillRunning

C=ROOT/'results/campaign_v3_20261030'

def build_plan():
    if (C/'PLAN.json').exists():raise FileExistsError('frozen v3 plan exists')
    old=read(ROOT/'config/experiment_plan.json');p=copy.deepcopy(old)
    p['schema_version']=3;p['evaluation_run_count']=6;p['evaluation_placement_count']=2
    p['trials']=[];p['placements']={k:v for k,v in old['placements'].items() if v['split']!='evaluation'}
    for side,y in [('left',.0215),('right',-.0215)]:
        key='v3_eval_'+side
        p['placements'][key]={'split':'evaluation','world_bottom_xyz':[.22,y,1.015],'yaw_rad':0.,'needle_label':'abnormal','needle_angle_rad':.875}
    # Reject exact historical coordinates, including old held-out examples.
    coords={tuple(v['world_bottom_xyz']) for v in old['placements'].values()}
    hist=read(ROOT/'placement_design.json')['historical_placement_inventory']
    def lists(value):
        if isinstance(value,dict):
            for v in value.values():yield from lists(v)
        elif isinstance(value,list):
            if len(value)==3 and all(isinstance(x,(int,float)) for x in value):yield tuple(value)
            for v in value:yield from lists(v)
    coords.update(lists(hist))
    assert all(tuple(v['world_bottom_xyz']) not in coords for k,v in p['placements'].items() if v['split']=='evaluation')
    prep=[('home','range_train_left_large','fixed'),('left','range_train_left_large','rule'),('left_changed','range_train_left_large','rule'),('right','range_train_right_large','rule')]
    for kind,pid,mode in prep:p['trials'].append({'run_id':'ire3_'+kind+'_20261030_01','kind':kind,'phase':'path_probe','placement_id':pid,'mode':mode})
    for phase in ['training','tuning']:
        for i,t in enumerate(x for x in old['trials'] if x['phase']==phase):
            p['trials'].append({**t,'run_id':'ire3_'+phase+'_'+str(i+1).zfill(2),'kind':'task'})
    for pid in ['v3_eval_left','v3_eval_right']:
        for mode in ['fixed','rule','clone']:
            p['trials'].append({'run_id':'ire3_'+pid+'_'+mode,'kind':'task','phase':'evaluation','placement_id':pid,'mode':mode})
    for phase,key in [('training','training_run_ids'),('tuning','tuning_run_ids'),('evaluation','evaluation_run_ids')]:p[key]=[t['run_id'] for t in p['trials'] if t['phase']==phase]
    p['approved_limits']={'existing_consumed':1,'global':28,'main':20,'optional_preparation_max':4}
    write(C/'PLAN.json',p)
    write(C/'plan_lock.json',{'plan_sha256':digest(C/'PLAN.json'),'image_control_sha256':digest(ROOT/'config/image_control.json'),'requested_command_cap_rad':.055,'safety_cap_rad':.055})
    print(json.dumps({'status':'PLAN_FROZEN_NO_MOTION','training':12,'tuning':2,'evaluation':6,'evaluation_placements':2,'sha256':digest(C/'PLAN.json')}))

def states():
    result={}
    if (C/'progress_events.jsonl').exists():
        for line in (C/'progress_events.jsonl').read_text(encoding='utf-8').splitlines():
            e=json.loads(line)
            if result.get(e['run_id'],{}).get('status') in ['PASS','QUALITY_FAIL','FAULT_STOP','PROTOCOL_STOP','INTERRUPTED']:raise RuntimeError('terminal overwrite')
            result[e['run_id']]=e
    return result

def commit(t,status,**extra):
    append(C/'progress_events.jsonl',{'run_id':t['run_id'],'trial':t,'status':status,'at_utc':now(),**extra})
    s=states();write(C/'progress_ledger.json',{'updated_at_utc':now(),'prior_consumed':1,'additional_consumed':len(s),'consumed_total':1+len(s),'remaining':27-len(s),'states':s})

def expected_q():
    from diagnostics.direct_route import last_direct_command
    last=last_direct_command()
    if last['result']!='VERIFIED_COMPLETED_DIRECT_COMMAND':raise RuntimeError('latest command completion unknown')
    return last['entry']['arrival']['actual_positions']

def run_one(t,fd,env):
    from datetime import datetime,timezone
    if datetime.now(timezone.utc)>=datetime.fromisoformat(read(C/'AUTHORIZATION.json')['last_new_experiment_start_jst']):raise RuntimeError('new experiment cutoff reached')
    s=states()
    if t['run_id'] in s or (C/'runs'/t['run_id']).exists():raise RuntimeError('no replay of completed/failed/reserved trial')
    if 1+len(s)>=28:raise RuntimeError('global consumed budget exhausted')
    # A process crash after reservation remains consumed and blocks automatic replay.
    if any(e['status']=='RESERVED' for e in s.values()):raise RuntimeError('prior active/interrupted run requires review')
    rd=C/'runs'/t['run_id'];rd.mkdir(parents=True,exist_ok=False)
    write(rd/'reservation.json',{'trial':t,'plan_sha256':digest(C/'PLAN.json'),'at_utc':now()});commit(t,'RESERVED',directory=str(rd))
    dash=rec=None;handles=[]
    try:
        env=dict(env);env['RANGE_TRIAL_STARTED_MONOTONIC']=str(time.monotonic())
        env=source_environment(rd,env,fd);env['RANGE_CAMPAIGN_RUNNER_PID']=str(os.getpid())
        q=expected_q();p=read(C/'PLAN.json')
        if t['kind']=='home':
            old=read(ROOT/'results/campaign/runs/ire2_probe_train_left_large_rule_01/setup_receipt.json')
            safety=Path(old['private_safety_config'])
            if digest(safety)!=old['private_safety_sha256']:raise RuntimeError('original verified private safety changed')
            receipt={'run_id':t['run_id'],'private_safety_sha256':digest(safety),'private_safety_config':str(safety)}
        else:
            from prepare_episode import prepare_episode
            owner=C/'target_ownership.json'
            if not owner.exists():write(owner,read(ROOT/'results/campaign/target_ownership.json'))
            oldenv=dict(os.environ)
            try:
                os.environ.clear();os.environ.update(env);safety=prepare_episode(p,t,rd,C)
            finally:os.environ.clear();os.environ.update(oldenv)
            receipt=read(rd/'setup_receipt.json')
        receipt['expected_initial_positions_rad']=q;write(rd/'setup_receipt.json',receipt)
        env['ACTIVE_SAFETY_CONFIG']=str(safety);env.setdefault('DISPLAY',':0');env.setdefault('XAUTHORITY','/home/ubuntu/.Xauthority')
        handles=[(rd/'dashboard.log').open('x'),(rd/'recording.log').open('x')]
        dash=subprocess.Popen([sys.executable,str(ROOT/'dashboard.py'),'--run-dir',str(rd)],env=env,stdout=handles[0],stderr=subprocess.STDOUT)
        rec=subprocess.Popen(['ffmpeg','-n','-nostdin','-f','x11grab','-video_size','1920x1080','-framerate','20','-i',':0.0','-c:v','libx264','-preset','ultrafast','-crf','24','-pix_fmt','yuv420p',str(rd/'trial_screen.mp4')],env=env,stdout=handles[1],stderr=subprocess.STDOUT)
        time.sleep(1)
        if rec.poll() is not None or dash.poll() is not None:raise RuntimeError('owned media did not start')
        write(rd/'owned_processes.json',{'recorder_pid':rec.pid,'dashboard_pid':dash.pid})
        command([sys.executable,str(ROOT/'live_v3.py'),'--run-id',t['run_id']],rd,'controller',env,fd,320)
        m=read(rd/'run_manifest.json')
        if t['kind']!='home':
            truth=read(rd/'setup_truth.json');label=(m.get('classification') or {}).get('label')
            write(rd/'score.json',{'run_id':t['run_id'],'expected_label':truth['truth_label'],'predicted_label':label,
                'capture_success':m.get('classification') is not None,'classification_correct':label==truth['truth_label'] if label else None,
                'home_return':(m.get('home_return') or {}).get('result'),'whole_task_success':m['result']=='PASS' and label==truth['truth_label'],
                'model_call_count':m['model_call_count'],'correction_count':m['alignment_trajectory_publish_count'],'all_trajectory_publish_count':m['all_trajectory_publish_count']})
        commit(t,m['result'],manifest=str(rd/'run_manifest.json'),home=(m.get('home_return') or {}).get('result'))
        print(json.dumps({'run_id':t['run_id'],'result':m['result'],'published':m['all_trajectory_publish_count'],'home':m.get('home_return',{}).get('result') if m.get('home_return') else None}),flush=True)
        return m
    except BaseException as exc:
        if states().get(t['run_id'],{}).get('status')=='RESERVED':commit(t,'FAULT_STOP',reason=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        if not (rd/'controller_still_running.json').exists():
            stop_media(rec,rd,'recorder');stop_media(dash,rd,'dashboard')
        for h in handles:h.close()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--build-plan',action='store_true');ap.add_argument('--kind',choices=['home','left','left_changed','right','training','tuning','evaluation']);args=ap.parse_args()
    if args.build_plan:return build_plan()
    if not args.kind:print(json.dumps({'status':'NO_MOTION','states':states()}));return
    if os.name!='posix':raise RuntimeError('execute only in retained container')
    assert_environment()
    if read(C/'READINESS_SAVED_DATA_REASSESSMENT.json')['result']!='PASS_FOR_KNOWN_LOGGED_DIRECT_ROUTE_AT_OBSERVATION_TIME':raise RuntimeError('route readiness missing')
    deadline=__import__('datetime').datetime.fromisoformat(read(C/'AUTHORIZATION.json')['last_new_experiment_start_jst'])
    if __import__('datetime').datetime.now(__import__('datetime').timezone.utc)>=deadline:raise RuntimeError('new experiment cutoff reached')
    p=read(C/'PLAN.json')
    if (C/'CAMPAIGN_STOP.json').exists():raise RuntimeError('campaign stop retained; no automatic resumption')
    blocked=[e for e in states().values() if e['status'] in ['FAULT_STOP','PROTOCOL_STOP','INTERRUPTED']]
    review=read(C/'LEFT_CHANGE_APPROVED_EVIDENCE.json') if (C/'LEFT_CHANGE_APPROVED_EVIDENCE.json').exists() else None
    reviewed={review['reviewed_stop_run_id']} if review and (args.kind=='left_changed' or states().get('ire3_left_changed_20261030_01',{}).get('status')=='PASS') else set()
    if any(e['run_id'] not in reviewed for e in blocked):raise RuntimeError('unreviewed motion fault; no later trial')
    if args.kind in ['training','tuning','evaluation']:
        contract=read(C/'COLLECTION_CONTRACT.json')
        for row in contract['files']:
            if digest(row['path'])!=row['sha256']:raise RuntimeError('collection conditions changed')
    if args.kind in ['tuning','evaluation']:
        if any(states().get(rid,{}).get('status') not in ['PASS','QUALITY_FAIL'] and rid not in reviewed for rid in p['training_run_ids']):raise RuntimeError('training attempts not complete')
    if args.kind=='evaluation':
        if any(states().get(rid,{}).get('status') not in ['PASS','QUALITY_FAIL'] for rid in p['tuning_run_ids']):raise RuntimeError('tuning attempts not complete')
    selected=[t for t in p['trials'] if t['kind']==args.kind] if args.kind in ['home','left','left_changed','right'] else [t for t in p['trials'] if t['phase']==args.kind]
    with FileLease(ROOT/'results/campaign/runner.lock'),FileLease(C/'runner.lock'),FileLease('/tmp/physicalai-active-inspection-trial.lock') as lease:
        fd=lease.file.fileno();env=dict(os.environ);env['ACTIVE_INSPECTION_LOCK_FD']=str(fd)
        for t in selected:
            if t['run_id'] in states():continue
            s=states()
            if args.kind=='home' and s:raise RuntimeError('home recovery only before task runs')
            if args.kind in ['left','left_changed','right','training','tuning','evaluation']:
                # Mandatory predecessors: don't turn performance failures into safety passes.
                if args.kind=='left' and s.get('ire3_home_20261030_01',{}).get('status')!='PASS':raise RuntimeError('home recovery prerequisite not confirmed')
                if args.kind=='left_changed' and not (C/'LEFT_CHANGE_APPROVED_EVIDENCE.json').exists():raise RuntimeError('identified unsent rejection and offline fix evidence required')
                if args.kind=='right' and not any(s.get('ire3_'+k+'_20261030_01',{}).get('status')=='PASS' for k in ['left','left_changed']):raise RuntimeError('left prerequisite missing')
                if args.kind in ['training','tuning','evaluation'] and s.get('ire3_right_20261030_01',{}).get('status')!='PASS':raise RuntimeError('both directions not verified')
            try:m=run_one(t,fd,env)
            except OwnedChildStillRunning:lease.defer_unlock=True;raise
            if m['result'] not in ['PASS','QUALITY_FAIL']:break
            last=list(states().values())[-2:]
            if len(last)==2 and all(e['status']=='QUALITY_FAIL' for e in last):
                write(C/'CAMPAIGN_STOP.json',{'reason':'two consecutive capture quality failures','at_utc':now()});break

if __name__=='__main__':main()
