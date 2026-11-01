"""Bounded sequential campaign. Default is read-only status; no ROS imports."""
from __future__ import annotations
from pathlib import Path
import argparse,json,os,signal,subprocess,sys,time
from common import ROOT,ACTIVE,PLAN,read,write,append,digest,now,validate_plan,assert_environment
from ledger import FileLease,Ledger,TERMINAL

CAMPAIGN=ROOT/'results/campaign'

def saved_reviewed_stops(campaign_root,plan_sha):
    path=Path(campaign_root)/'stop_review_acknowledgements.jsonl';result=set()
    if path.exists():
        for line in path.read_text(encoding='utf-8').splitlines():
            e=json.loads(line)
            if e['plan_sha256']!=plan_sha:raise RuntimeError('stop review belongs to another plan')
            result.update(e['reviewed_stop_run_ids'])
    return result

def checked_plan(path=PLAN):
    plan=validate_plan(read(path));lock=read(ROOT/'config/plan_lock.json')
    if digest(path)!=lock['plan_sha256'] or digest(ROOT/'placement_design.json')!=lock['placement_design_sha256'] or digest(ROOT/'config/image_control.json')!=lock['image_control_sha256']:raise RuntimeError('frozen plan or image configuration changed')
    if 'source_contract_sha256' in lock:
        contract=ROOT/'config/source_contract.json'
        if digest(contract)!=lock['source_contract_sha256']:raise RuntimeError('retained source contract changed')
        for item in read(contract)['files']:
            if digest(ACTIVE.parent/item['repository_relative_path'])!=item['sha256']:raise RuntimeError('retained source/config/model changed: '+item['repository_relative_path'])
    if digest(ACTIVE/'artifacts/image_jacobian.json')!=plan['teacher']['calibration_sha256']:raise RuntimeError('frozen measured teacher calibration changed')
    return plan

def check_authorization(path,plan_sha,phase):
    if path is None:raise RuntimeError('motion is not authorized by this preparation turn; a separately approved execution authorization file is required')
    if not Path(path).resolve().is_relative_to((ROOT/'config/authorizations').resolve()):raise RuntimeError('execution authorization must be a separately approved file under config/authorizations, never an offline fixture')
    a=read(path)
    if a.get('motion_execution_approved') is not True or a.get('plan_sha256')!=plan_sha or phase not in a.get('allowed_phases',[]):raise RuntimeError('authorization does not cover this exact plan and phase')
    if a.get('max_total_trials')!=28 or a.get('max_main_trials')!=26 or a.get('max_path_probes')!=2:raise RuntimeError('authorization budget differs')
    return a

def check_prerequisites(plan,ledger,phase):
    if phase!='path_probe':
        probes=[t for t in plan['trials'] if t['phase']=='path_probe']
        for t in probes:
            e=ledger.states.get(t['run_id'],{})
            if e.get('status') not in ['PASS','QUALITY_FAIL'] or not e.get('path_arrival_image_home_pass'):raise RuntimeError('both path probes require verified path/arrival/fresh image/home first')
    if phase in ['tuning','evaluation']:
        for rid in plan['training_run_ids']:
            if ledger.states.get(rid,{}).get('status') not in TERMINAL:raise RuntimeError('training attempts not all terminal')
    if phase=='evaluation':
        for rid in plan['tuning_run_ids']:
            if ledger.states.get(rid,{}).get('status') not in TERMINAL:raise RuntimeError('tuning attempts not all terminal')

def frozen_model(plan_sha):
    lock=read(ROOT/'artifacts/evaluation_lock.json')
    model=Path(lock['model_path']);meta=model.with_suffix('.json')
    if lock['plan_sha256']!=plan_sha or digest(model)!=lock['model_sha256'] or digest(meta)!=lock['metadata_sha256']:raise RuntimeError('frozen evaluation model/plan mismatch')
    if digest(ROOT/'policy.py')!=lock['policy_program_sha256']:raise RuntimeError('policy code changed after evaluation freeze')
    for item in lock['execution_files']:
        if digest(Path(item['path']))!=item['sha256']:raise RuntimeError('execution code/settings changed after evaluation freeze')
    return model,lock['model_sha256']

class OwnedChildStillRunning(RuntimeError):
    def __init__(self,pid,message,interrupted=False):super().__init__(message);self.pid=pid;self.interrupted=interrupted

def command(argv,rd,name,env,fd,timeout):
    log=rd/'runner_commands.jsonl';start=time.monotonic()
    append(log,{'stage':'STARTED','at_utc':now(),'argv':argv,'timeout_sec':timeout,'retries':0})
    with (rd/(name+'.stdout.log')).open('x') as out,(rd/(name+'.stderr.log')).open('x') as err:
        proc=subprocess.Popen(argv,env=env,pass_fds=(fd,),stdout=out,stderr=err)
        try:rc=proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            append(log,{'stage':'TIMEOUT_CHILD_NOT_KILLED','at_utc':now(),'pid':proc.pid,'elapsed_sec':time.monotonic()-start,'automatic_resend':False})
            write(rd/(name+'_still_running.json'),{'pid':proc.pid,'at_utc':now(),'executor_lease_inherited':True,'automatic_kill':False})
            raise OwnedChildStillRunning(proc.pid,'owned client exceeded watchdog; no kill/retry/motion recovery; inherited executor lease retained')
        except KeyboardInterrupt:
            if proc.poll() is None:
                write(rd/(name+'_still_running.json'),{'pid':proc.pid,'at_utc':now(),'executor_lease_inherited':True,'automatic_kill':False,'parent_interrupted':True})
                raise OwnedChildStillRunning(proc.pid,'parent interrupted; owned child may remain; inherited executor lease retained',interrupted=True)
            raise
    append(log,{'stage':'FINISHED','at_utc':now(),'argv':argv,'returncode':rc,'elapsed_sec':time.monotonic()-start})
    return rc

def stop_media(process,rd,name):
    if process is None:return
    if process.poll() is None:
        process.send_signal(signal.SIGINT if name=='recorder' else signal.SIGTERM)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            write(rd/(name+'_still_running.json'),{'pid':process.pid,'at_utc':now(),'no_sigkill':True})
            raise OwnedChildStillRunning(process.pid,'owned media child failed to exit; no broad process stop')

def source_environment(rd,env,fd):
    argv=['bash',str(ROOT/'diagnostics/source_ros_environment.sh')];start=time.monotonic()
    append(rd/'runner_commands.jsonl',{'stage':'SOURCE_ENVIRONMENT_STARTED','at_utc':now(),'argv':argv,'timeout_sec':15,'full_environment_logged':False})
    try:
        p=subprocess.run(argv,env=env,pass_fds=(fd,),stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=15)
        log=p.stderr.decode(errors='replace')
        (rd/'source_environment.log').write_text(log,encoding='utf-8')
        if p.returncode:raise RuntimeError('ROS setup command failed; no retry')
        sourced=dict(x.split('=',1) for x in p.stdout.decode().split('\0') if '=' in x)
        keys=['ROS_DOMAIN_ID','IGN_PARTITION','GZ_PARTITION']
        if any(sourced.get(k)!=env.get(k) for k in keys):raise RuntimeError('domain or partition changed during setup; stopped')
        append(rd/'runner_commands.jsonl',{'stage':'SOURCE_ENVIRONMENT_FINISHED','at_utc':now(),'elapsed_sec':time.monotonic()-start,'domain_partition_unchanged':True})
        return sourced
    except subprocess.TimeoutExpired as e:
        (rd/'source_environment.log').write_text((e.stderr or b'').decode(errors='replace'),encoding='utf-8')
        append(rd/'runner_commands.jsonl',{'stage':'SOURCE_ENVIRONMENT_TIMEOUT','at_utc':now(),'elapsed_sec':time.monotonic()-start,'automatic_retry':False})
        raise

def execute_trial(plan,t,rd,campaign_root,env,fd,model_info=None):
    from prepare_episode import prepare_episode
    began=time.monotonic();env=source_environment(rd,env,fd)
    diag=ROOT/'results/diagnostics'/f"staged_readiness_{t['run_id']}.json"
    rc=command([sys.executable,str(ROOT/'diagnostics/staged_readiness.py'),'--label',t['run_id'],'--output',str(diag)],rd,'readiness',env,fd,100)
    gate=read(diag)
    if gate.get('diagnostic_worker_still_running'):raise OwnedChildStillRunning(-1,'read-only diagnostic child still present')
    if rc or gate.get('gate_result')!='PASS':raise RuntimeError('readiness unknown/abnormal; no target mutation or movement')
    oldenv=dict(os.environ)
    try:
        os.environ.clear();os.environ.update(env)
        safety=prepare_episode(plan,t,rd,campaign_root)
    finally:
        os.environ.clear();os.environ.update(oldenv)
    env=dict(env);env['ACTIVE_SAFETY_CONFIG']=str(safety);env.setdefault('DISPLAY',':0');env.setdefault('XAUTHORITY','/home/ubuntu/.Xauthority')
    dash=None;recorder=None;handles=[]
    try:
        for name in ['dashboard','recording']:handles.append((rd/(name+'.log')).open('x'))
        dash=subprocess.Popen([sys.executable,str(ROOT/'dashboard.py'),'--run-dir',str(rd)],env=env,stdout=handles[0],stderr=subprocess.STDOUT)
        time.sleep(1)
        if dash.poll() is not None:raise RuntimeError('dashboard failed before control')
        recorder=subprocess.Popen(['ffmpeg','-n','-nostdin','-f','x11grab','-video_size','1920x1080','-framerate','20','-i',':0.0','-c:v','libx264','-preset','ultrafast','-crf','24','-pix_fmt','yuv420p',str(rd/'trial_screen.mp4')],env=env,stdout=handles[1],stderr=subprocess.STDOUT)
        time.sleep(1)
        if recorder.poll() is not None:raise RuntimeError('recording failed before control')
        write(rd/'owned_media_processes.json',{'at_utc':now(),'dashboard_pid':dash.pid,'recorder_pid':recorder.pid,'state':'RUNNING','ownership':'Only children started by this run; simulator and controller processes excluded.'})
        argv=[sys.executable,str(ROOT/'runtime.py'),'--execute','--run-id',t['run_id'],'--mode',t['mode'],'--phase',t['phase'],'--run-dir',str(rd)]
        if t['mode']=='clone':argv+=['--model',str(model_info[0]),'--expected-model-sha256',model_info[1]]
        rc=command(argv,rd,'controller',env,fd,380)
        m=read(rd/'run_manifest.json')
        if recorder.poll() is not None or dash.poll() is not None:raise RuntimeError('media/display exited during trial; recorded task manifest retained')
        if rc and m['result'] in ['PASS','QUALITY_FAIL']:raise RuntimeError('controller exit/result mismatch')
    finally:
        # Only our own recorder/UI are signalled; never the simulator/controller.
        if 'm' in locals() or not (rd/'controller_still_running.json').exists():
            stop_media(recorder,rd,'recorder');stop_media(dash,rd,'dashboard')
        for h in handles:h.close()
    if not (rd/'trial_screen.mp4').exists():raise RuntimeError('raw recording missing')
    m['episode_total_elapsed_sec']=time.monotonic()-began
    write(rd/'run_manifest.json',m)
    truth=read(rd/'setup_truth.json');pred=(m.get('classification') or {}).get('label')
    write(rd/'score.json',{'run_id':t['run_id'],'expected_label':truth['truth_label'],'predicted_label':pred,'classification_correct':pred==truth['truth_label'] if pred else None,'capture_success':m.get('classification') is not None,'home_return':(m.get('home_return') or {}).get('result'),'whole_task_success':m['result']=='PASS' and pred==truth['truth_label'],'result':m['result']})
    return m

def run_sequence(plan,ledger,phase,execute,reviewed_stops=frozenset()):
    """Pure injectable sequencing used by both the real runner and offline QA."""
    if any(e['run_id'] not in reviewed_stops for e in ledger.blocking()):raise RuntimeError('unreviewed campaign stop; no remaining trial started')
    check_prerequisites(plan,ledger,phase)
    done=[]
    for t in ledger.pending(phase):
        rd=ledger.reserve(t)
        try:
            m=execute(t,rd)
            status=m['result']
            if status not in TERMINAL:raise RuntimeError('controller result is not terminal')
            ledger.finish(t['run_id'],status,manifest=str(rd/'run_manifest.json'),home_return=(m.get('home_return') or {}).get('result'),path_arrival_image_home_pass=m.get('path_arrival_image_home_pass',False),model_call_count=m.get('model_call_count',0),teacher_action_records=m.get('teacher_action_records',0))
            done.append(t['run_id'])
            if status not in ['PASS','QUALITY_FAIL']:break
        except BaseException as e:
            if ledger.states[t['run_id']]['status']=='RESERVED':ledger.finish(t['run_id'],'INTERRUPTED' if isinstance(e,(KeyboardInterrupt,SystemExit)) or getattr(e,'interrupted',False) else 'FAULT_STOP',reason=f'{type(e).__name__}: {e}',owned_child_pid=getattr(e,'pid',None))
            raise
    return done

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--phase',choices=['path_probe','training','tuning','evaluation'],default='path_probe')
    ap.add_argument('--execute',action='store_true');ap.add_argument('--initialize',action='store_true')
    ap.add_argument('--authorization',type=Path);args=ap.parse_args()
    plan=checked_plan();plan_sha=digest(PLAN)
    if args.execute:check_authorization(args.authorization,plan_sha,args.phase)
    with FileLease(CAMPAIGN/'runner.lock'):
        ledger=Ledger(CAMPAIGN,plan,plan_sha)
        if not args.execute:
            if args.initialize:
                if ledger.snapshot.exists():raise FileExistsError('campaign already initialized; status only')
                write(ledger.snapshot,{'plan_sha256':plan_sha,'consumed_trials':0,'main_trials_consumed':0,'path_probes_consumed':0,'states':{},'status':'PREPARED_NOT_EXECUTED'})
            print(json.dumps({'status':'NO_MOTION','plan_sha256':plan_sha,'consumed':len(ledger.states),'pending_in_phase':len(ledger.pending(args.phase)),'model_retraining_started':False}));return 0
        if os.name!='posix':raise RuntimeError('motion campaign must execute only inside existing Linux container')
        assert_environment()
        ledger.recover_interrupted()
        authorization=read(args.authorization)
        new_reviewed=set(authorization.get('reviewed_stop_run_ids',[])) if authorization.get('resume_without_replay') is True else set()
        if new_reviewed-{e['run_id'] for e in ledger.blocking()}:raise RuntimeError('a review cannot authorize a future unknown fault')
        reviewed=saved_reviewed_stops(CAMPAIGN,plan_sha)|new_reviewed
        if any(e['run_id'] not in reviewed for e in ledger.blocking()):raise RuntimeError('campaign has an unreviewed fault/protocol/interruption stop; no replay')
        if new_reviewed:append(CAMPAIGN/'stop_review_acknowledgements.jsonl',{'at_utc':now(),'plan_sha256':plan_sha,'authorization_sha256':digest(args.authorization),'reviewed_stop_run_ids':sorted(new_reviewed),'old_terminal_records_unchanged':True})
        model_info=frozen_model(plan_sha) if args.phase=='evaluation' else None
        with FileLease('/tmp/physicalai-active-inspection-trial.lock') as shared:
            fd=shared.file.fileno();env=dict(os.environ);env['ACTIVE_INSPECTION_LOCK_FD']=str(fd);env['RANGE_CAMPAIGN_RUNNER_PID']=str(os.getpid())
            # Setup helpers execute in this same process and also require the lease.
            inherited=os.environ.get('ACTIVE_INSPECTION_LOCK_FD');os.environ['ACTIVE_INSPECTION_LOCK_FD']=str(fd)
            def execute(t,rd):
                if args.phase=='evaluation':frozen_model(plan_sha)
                return execute_trial(plan,t,rd,CAMPAIGN,env,fd,model_info)
            try:run_sequence(plan,ledger,args.phase,execute,reviewed)
            except OwnedChildStillRunning:
                shared.defer_unlock=True;raise
            finally:
                if inherited is None:os.environ.pop('ACTIVE_INSPECTION_LOCK_FD',None)
                else:os.environ['ACTIVE_INSPECTION_LOCK_FD']=inherited
    return 0

if __name__=='__main__':raise SystemExit(main())
