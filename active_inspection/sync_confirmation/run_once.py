"""Exactly one separately authorized reproduction run; frozen controller/model."""
from pathlib import Path
import os,sys,argparse,time,subprocess,signal,json,inspect
ACTIVE=Path(__file__).resolve().parents[1];R=ACTIVE/'range_extension';sys.path.insert(0,str(R))
from common import read,write,append,digest,now,assert_environment
from ledger import FileLease
from runner import source_environment,command,stop_media,OwnedChildStillRunning
C=R/'results/campaign_v3_20261030';OUT=R/'results/sync_confirmation_20261030_01'
RID='ire3_repro_sync_20261030_01';RD=C/'runs'/RID

def recording_ok():
    h=read(RD/'continuous/heartbeat.json')
    if not h['ready'] or h['errors'] or time.monotonic()-h['monotonic']>1.5:raise RuntimeError('continuous image/pose/joint/GUI recording not fresh; no new command')
    return h

def worker():
    from runtime import load_file,real_observe,real_classify
    from controller import run_controller,new_manifest
    from policy import Joint1ClonePolicy
    from live_v3 import RequestedHeadroom
    from motion_binding import bind_motion
    assert_environment()
    if not os.environ.get('ACTIVE_INSPECTION_LOCK_FD') or os.environ.get('SYNC_CONFIRMATION_PARENT')!=str(os.getppid()):raise RuntimeError('authorized parent lock absent')
    if (RD/'run_manifest.json').exists():raise FileExistsError('no replay')
    receipt=read(RD/'setup_receipt.json');cfgpath=Path(os.environ['ACTIVE_SAFETY_CONFIG'])
    if digest(cfgpath)!=receipt['private_safety_sha256']:raise RuntimeError('safety receipt differs')
    frozen=read(C/'evaluation_lock.json')
    for r in frozen['files']:
        if digest(Path(r['path']))!=r['sha256']:raise RuntimeError('frozen file differs')
    actor=RequestedHeadroom(Joint1ClonePolicy(Path(frozen['model_path'])),.0548)
    mod=load_file('sync_retained_motion',ACTIVE/'motion.py')
    if Path(inspect.getfile(mod.Motion)).resolve()!=(ACTIVE/'motion.py').resolve():raise RuntimeError('wrong Motion before ROS init')
    cfg=__import__('yaml').safe_load(cfgpath.read_text())
    if cfg['ros_domain_id']!=47 or cfg['partition']!='physicalai_active_47':raise RuntimeError('wrong motion configuration domain')
    settings=read(R/'config/image_control.json');m=new_manifest(RID,'clone','reproduction_recording_confirmation',digest(R/'config/image_control.json'),actor)
    m.update(motion_import_source_absolute=str(ACTIVE/'motion.py'),actual_environment=assert_environment(),not_part_of_final_evaluation=True)
    write(RD/'run_manifest.json',m);lease=load_file('sync_legacy_lease',ACTIVE/'controller_lock.py');fd=None;owned=False;motion=None
    Base=bind_motion(mod)
    class RecordedMotion(Base):
        def check_publisher_ownership(self):recording_ok();return super().check_publisher_ownership()
    try:
        fd,owned=lease.acquire();recording_ok();mod.rclpy.init();motion=RecordedMotion();motion.wait_controller_ready()
        expected=receipt['expected_initial_positions_rad'];began=time.monotonic();samples=[]
        while time.monotonic()-began<.55:
            q=motion.wait_sample();v=dict(zip(motion.latest.name,motion.latest.velocity));samples.append({'positions':q,'header':mod.asdict(motion.latest.header),'at_utc':now()})
            if max(abs(x-y) for x,y in zip(q,expected))>.002 or any(abs(v[n])>.01 for n in motion.names):raise RuntimeError('initial measured state changed/moving')
        m['fresh_start_samples']=samples
        run_controller(motion,RD,m,settings,actor,real_observe,real_classify,started_at=float(os.environ['RANGE_TRIAL_STARTED_MONOTONIC']))
    except BaseException as e:m.update(result='FAULT_STOP',stage='ERROR',failure_reason=f'{type(e).__name__}: {e}',finished_at_utc=now());write(RD/'run_manifest.json',m)
    finally:
        if motion:motion.destroy_node()
        if mod.rclpy.ok():mod.rclpy.shutdown()
        lease.release(fd,owned)
    return 0 if m['result'] in ('PASS','QUALITY_FAIL') else 2

def execute():
    assert_environment()
    if RD.exists() or (OUT/'progress_events.jsonl').exists():raise FileExistsError('confirmation already reserved/completed/failed; never repeat')
    if read(OUT/'PREFLIGHT_RECORDING_GATE.json')['result']!='PASS':raise RuntimeError('passive recording method not confirmed')
    if read(C/'progress_ledger.json')['consumed_total']!=25:raise RuntimeError('base trial count changed')
    if read(OUT/'AUTHORIZATION.json')['maximum_additional_task_trials']!=1:raise RuntimeError('authorization differs')
    with FileLease(R/'results/campaign/runner.lock'),FileLease(C/'runner.lock'),FileLease(OUT/'runner.lock'),FileLease('/tmp/physicalai-active-inspection-trial.lock') as lock:
        fd=lock.file.fileno();env=dict(os.environ);env['ACTIVE_INSPECTION_LOCK_FD']=str(fd)
        setupdir=OUT/'execution_start';setupdir.mkdir(exist_ok=False);env=source_environment(setupdir,env,fd)
        env['ACTIVE_SAFETY_CONFIG']=str(C/'runs/ire3_v3_eval_right_clone/private_setup/safety_motion.yaml')
        gate=R/'results/diagnostics/staged_readiness_sync_confirmation_20261030_01.json'
        rc=command([sys.executable,str(R/'diagnostics/staged_readiness.py'),'--label','sync_confirmation_20261030_01','--output',str(gate),'--recovery-state'],setupdir,'readiness',env,fd,100)
        g=read(gate)
        if rc or g['gate_result']!='PASS' or g.get('at_home') is not True:raise RuntimeError('current readiness/home not confirmed; no target change/motion')
        from diagnostics.direct_route import last_direct_command,sender_processes
        history=last_direct_command()
        if history['result']!='VERIFIED_COMPLETED_DIRECT_COMMAND' or sender_processes():raise RuntimeError('known command history/executor abnormal')
        q=history['entry']['arrival']['actual_positions'];RD.mkdir(exist_ok=False)
        trial={'run_id':RID,'placement_id':'v3_eval_left','phase':'reproduction_recording_confirmation','mode':'clone'}
        write(RD/'reservation.json',{'trial':trial,'at_utc':now(),'base_total':25,'consumed_total':26,'not_final_evaluation':True})
        append(OUT/'progress_events.jsonl',{'run_id':RID,'status':'RESERVED','at_utc':now(),'consumed_total':26})
        write(OUT/'progress_ledger.json',{'base_consumed':25,'additional_consumed':1,'total_consumed':26,'remaining':2,'run_id':RID,'status':'RESERVED'})
        rec=screen=None;logs=[];unknown=False
        try:
            from prepare_episode import prepare_episode
            write(OUT/'target_ownership.json',read(C/'target_ownership.json'))
            prior=dict(os.environ)
            try:
                os.environ.clear();os.environ.update(env);safety=prepare_episode(read(C/'PLAN.json'),trial,RD,OUT)
            finally:os.environ.clear();os.environ.update(prior)
            receipt=read(RD/'setup_receipt.json');receipt['expected_initial_positions_rad']=q;write(RD/'setup_receipt.json',receipt)
            env['ACTIVE_SAFETY_CONFIG']=str(safety);env['SYNC_CONFIRMATION_PARENT']=str(os.getpid());env['RANGE_TRIAL_STARTED_MONOTONIC']=str(time.monotonic());env.setdefault('DISPLAY',':0');env.setdefault('XAUTHORITY','/home/ubuntu/.Xauthority')
            for name in ['continuous_recorder','external_recording']:logs.append((RD/(name+'.log')).open('x'))
            rec=subprocess.Popen([sys.executable,str(Path(__file__).with_name('recorder.py')),'--out',str(RD/'continuous')],env=env,stdout=logs[0],stderr=subprocess.STDOUT)
            screen=subprocess.Popen(['ffmpeg','-n','-nostdin','-f','x11grab','-video_size','1920x1080','-framerate','20','-i',':0.0','-c:v','libx264','-preset','ultrafast','-crf','20','-pix_fmt','yuv420p',str(RD/'external_continuous.mp4')],env=env,stdout=logs[1],stderr=subprocess.STDOUT)
            write(RD/'owned_recorders.json',{'recorder_pid':rec.pid,'screen_pid':screen.pid,'parent_pid':os.getpid()})
            end=time.monotonic()+25
            while time.monotonic()<end:
                if rec.poll() is not None or screen.poll() is not None:raise RuntimeError('recording process ended before motion')
                if (RD/'continuous/heartbeat.json').exists():
                    h=read(RD/'continuous/heartbeat.json')
                    if h['ready']:
                        write(RD/'PRE_MOTION_RECORDING_PROOF.json',h);break
                time.sleep(.1)
            else:raise RuntimeError('image/pose/provider recording not established; no motion')
            recording_ok();write(RD/'MOTION_WINDOW_START.json',{'at_utc':now(),'target_setup_complete':True,'no_further_target_mutation_permitted':True})
            rc=command([sys.executable,str(Path(__file__)),'--worker'],RD,'controller',env,fd,320)
            m=read(RD/'run_manifest.json');status=m['result'];time.sleep(1)
            write(RD/'MOTION_WINDOW_END.json',{'at_utc':now(),'result':status})
        except OwnedChildStillRunning:
            unknown=True;lock.defer_unlock=True;raise
        except BaseException as e:
            status='FAULT_STOP';write(RD/'confirmation_error.json',{'at_utc':now(),'error':repr(e),'no_retry':True});raise
        finally:
            if not unknown:
                stop_media(screen,RD,'screen_recorder');stop_media(rec,RD,'continuous_recorder')
                for f in logs:f.close()
                append(OUT/'progress_events.jsonl',{'run_id':RID,'status':status,'at_utc':now(),'consumed_total':26})
                write(OUT/'progress_ledger.json',{'base_consumed':25,'additional_consumed':1,'total_consumed':26,'remaining':2,'run_id':RID,'status':status,'directory':str(RD)})
        print(json.dumps({'run_id':RID,'status':status,'total_consumed':26,'remaining':2,'model_calls':m['model_call_count'],'all_sends':m['all_trajectory_publish_count'],'home':m['home_return']}))

def main():
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--worker',action='store_true');a=p.parse_args()
    if a.worker:return worker()
    if a.execute:return execute()
    print(json.dumps({'status':'NO_MOTION','scope':'one reproduction confirmation; original freeze and ledgers retained','hard_step':.055,'requested_cap':.0548,'model_hash':read(OUT/'AUTHORIZATION.json')['model_sha256']}))
if __name__=='__main__':raise SystemExit(main())
