"""One reserved v3 run only; uses retained Motion checks and controller."""
from pathlib import Path
import argparse,os,sys,time,inspect
from common import ROOT,ACTIVE,read,write,digest,now,assert_environment
from runtime import load_file,real_observe,real_classify
from controller import run_controller,new_manifest
from motion_binding import bind_motion
from policy import Joint1RuleTeacher,Joint1ClonePolicy


class RequestedHeadroom:
    def __init__(self,actor,cap):self.actor=actor;self.cap=cap;self.model_sha256=getattr(actor,'model_sha256',None)
    def predict_with_audit(self,*args):
        a=self.actor.predict_with_audit(*args)
        if a['result']=='PASS':
            hard=a['safety_limited_output_rad'][:]
            a['hard_safety_ceiling_output_rad']=hard
            a['requested_command_cap_rad']=self.cap
            a['safety_limited_output_rad']=[max(-self.cap,min(self.cap,hard[0])),0.,0.,0.]
        return a


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run-id',required=True);args=ap.parse_args()
    campaign=ROOT/'results/campaign_v3_20261030';rd=campaign/'runs'/args.run_id
    assert_environment()
    if os.environ.get('RANGE_CAMPAIGN_RUNNER_PID')!=str(os.getppid()) or not os.environ.get('ACTIVE_INSPECTION_LOCK_FD'):raise RuntimeError('authorized parent and inherited lock required')
    if (rd/'run_manifest.json').exists():raise FileExistsError('no replay')
    reservation=read(rd/'reservation.json');t=reservation['trial'];receipt=read(rd/'setup_receipt.json');lock=read(campaign/'plan_lock.json')
    if reservation['plan_sha256']!=lock['plan_sha256'] or receipt['run_id']!=args.run_id:raise RuntimeError('reservation/setup mismatch')
    cfgpath=Path(os.environ['ACTIVE_SAFETY_CONFIG'])
    if digest(cfgpath)!=receipt['private_safety_sha256']:raise RuntimeError('verified safety config differs')
    settings=read(ROOT/'config/image_control.json')
    if digest(ROOT/'config/image_control.json')!=lock['image_control_sha256']:raise RuntimeError('capture/control conditions changed')
    actor=Joint1RuleTeacher(ACTIVE/'artifacts/image_jacobian.json',gain=.5) if t['mode']=='rule' else None
    if t['mode']=='clone':
        ml=read(campaign/'evaluation_lock.json');model=Path(ml['model_path'])
        for row in ml['files']:
            if digest(row['path'])!=row['sha256']:raise RuntimeError('evaluation freeze changed')
        actor=Joint1ClonePolicy(model)
    if actor:actor=RequestedHeadroom(actor,float(lock['requested_command_cap_rad']))
    mod=load_file('v3_retained_motion',ACTIVE/'motion.py')
    if Path(inspect.getfile(mod.Motion)).resolve()!=(ACTIVE/'motion.py').resolve():raise RuntimeError('wrong Motion before ROS init')
    cfg=__import__('yaml').safe_load(cfgpath.read_text())
    if cfg['ros_domain_id']!=47 or cfg['partition']!='physicalai_active_47':raise RuntimeError('wrong config domain before ROS init')
    m=new_manifest(args.run_id,t['mode'],t['phase'],digest(ROOT/'config/image_control.json'),actor)
    m['motion_import_source_absolute']=str(ACTIVE/'motion.py');m['actual_environment']=assert_environment()
    if t['kind']=='home':m['controller']='independent_home_recovery'
    def persist():
        m['all_trajectory_publish_count']=sum(e.get('publish_count',0) for e in m['motions'])
        m['alignment_trajectory_publish_count']=sum(a.get('actual_publish_count',0) for a in m['actions'])
        m['unconfirmed_send_intent_command_ids']=[e['command_id'] for e in m['motions'] if e.get('publish_intent_at_utc') and not e.get('published_at_utc') and not e.get('publish_count')]
        write(rd/'run_manifest.json',m)
    persist();motion=None;lease=load_file('v3_lease',ACTIVE/'controller_lock.py');fd=None;owned=False
    try:
        fd,owned=lease.acquire();mod.rclpy.init();motion=bind_motion(mod)()
        # Fresh start checks are intrinsic to this reserved motion run, not a
        # third stage-1 diagnostic invocation. No trajectory has been sent.
        m['controller_readiness']=motion.wait_controller_ready()
        before=receipt['expected_initial_positions_rad'];samples=[];began=time.monotonic()
        while time.monotonic()-began<.55:
            q=motion.wait_sample();v=dict(zip(motion.latest.name,motion.latest.velocity))
            samples.append({'positions_rad':q,'velocities_rad_sec':[v[n] for n in motion.names],'at_utc':now(),'joint_state_header':mod.asdict(motion.latest.header)})
            if max(abs(x-y) for x,y in zip(q,before))>.002 or any(abs(v[n])>.01 for n in motion.names):raise RuntimeError('start state changed or moving; no command')
        m['fresh_start_samples']=samples;persist()
        started=float(os.environ['RANGE_TRIAL_STARTED_MONOTONIC'])
        if t['kind']=='home':
            motion.register_scene(rd/'collision_scene.json');m['stage']='RETURN_HOME';persist()
            motion.move('verified_recovery_start','home',rd,m,persist)
            m['home_return']=motion.confirm_home();m.update(result='PASS',stage='DONE',finished_at_utc=now());persist()
        else:
            if t['kind']=='left_changed' and max(abs(x-y) for x,y in zip(samples[-1]['positions_rad'],settings['control']['home']))>.02:
                # Separate changed run, only after confirmed unsent refusal and
                # fresh startup checks. Count this positioning move in its budget.
                motion.register_scene(rd/'collision_scene_initial.json')
                motion.move('verified_changed_run_start','home',rd,m,persist)
                m['initial_positioning_home']=motion.confirm_home();persist()
            run_controller(motion,rd,m,settings,actor,real_observe,real_classify,started_at=started)
    except BaseException as exc:
        m.update(result='FAULT_STOP',stage='ERROR',failure_reason=f'{type(exc).__name__}: {exc}',finished_at_utc=now());persist()
    finally:
        if motion:motion.destroy_node()
        if mod.rclpy.ok():mod.rclpy.shutdown()
        lease.release(fd,owned)
    return 0 if m['result'] in ('PASS','QUALITY_FAIL') else 1

if __name__=='__main__':raise SystemExit(main())
