"""Authorized ROS binding. Default invocation is a no-motion contract summary."""
from __future__ import annotations
from pathlib import Path
import argparse,importlib.util,inspect,json,os,subprocess,sys,time
from common import ACTIVE,REPO,ROOT,read,write,append,digest,assert_environment,now
from controller import run_controller,new_manifest,publication_ownership_ok
from policy import Joint1RuleTeacher,Joint1ClonePolicy
from trajectory_guard import HorizontalPlanAuditMixin
from motion_binding import bind_motion

def load_file(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    return module

def real_observe(motion,rd,ordinal,after,limits):
    import cv2
    detection=load_file('range_old_detection',ACTIVE/'detection.py')
    path=rd/f'camera_{ordinal:02d}.png'
    if path.exists():raise FileExistsError('camera image ordinal already used')
    cmd=['ros2','run','camera_inspection','capture_image','--topic',motion.cfg['camera_topic'],'--output',str(path),'--timeout-sec',str(motion.cfg['image_timeout_sec']),'--minimum-image-std',str(motion.cfg['minimum_image_std'])]
    started=now();append(rd/'capture_commands.jsonl',{'stage':'STARTED','at_utc':started,'argv':cmd,'attempt':1})
    proc=subprocess.run(cmd,capture_output=True,text=True,timeout=30)
    append(rd/'capture_commands.jsonl',{'stage':'FINISHED','at_utc':now(),'argv':cmd,'returncode':proc.returncode,'stderr':proc.stderr[-1000:],'attempt':1})
    if proc.returncode:raise RuntimeError('image acquisition fault; no retry: '+proc.stderr[-300:])
    meta=read(path.with_suffix('.json'))
    if digest(path)!=meta['image_sha256']:raise RuntimeError('camera image hash mismatch')
    if meta['received_at_utc']<=after['at_utc']:raise RuntimeError('image received before arrival')
    a,b=after['joint_state_header']['stamp'],meta['header_stamp']
    if (b['sec'],b['nanosec'])<=(a['sec'],a['nanosec']):raise RuntimeError('camera timestamp not newer than arrival')
    frame=cv2.imread(str(path));found=detection.detect(frame,limits)
    return {'source_image_path':str(path),'metadata':meta,'detection':found,'quality':detection.quality(found,limits)}

def real_classify(last,rd):
    import cv2
    det=load_file('range_old_detection',ACTIVE/'detection.py')
    vision=load_file('range_old_classifier',REPO/'multigauge/vision.py')
    registered=rd/'registered_for_svm.png'
    if registered.exists():raise FileExistsError('classification image already exists')
    if not cv2.imwrite(str(registered),det.register_for_legacy_model(cv2.imread(last['source_image_path']),last['detection'])):raise RuntimeError('registered image save failed')
    pred=vision.classify(registered)
    if pred['image_sha256']!=digest(registered):raise RuntimeError('classification image hash mismatch')
    return {'label':pred['label'],'raw_camera_image_sha256':last['metadata']['image_sha256'],'inference':pred,'capture_quality':'PASS'}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true');ap.add_argument('--run-id');ap.add_argument('--mode',choices=['fixed','rule','clone'])
    ap.add_argument('--phase',choices=['path_probe','training','tuning','evaluation']);ap.add_argument('--run-dir',type=Path)
    ap.add_argument('--control',type=Path,default=ROOT/'config/image_control.json');ap.add_argument('--model',type=Path)
    ap.add_argument('--expected-model-sha256');args=ap.parse_args()
    if not args.execute:
        print(json.dumps({'status':'NO_MOTION','axis':'horizontal','joint_outputs':[1],'safety_step_rad':.055,'model_fallback':False}));return 0
    if not all([args.run_id,args.mode,args.phase,args.run_dir]):ap.error('execution requires full episode arguments')
    # A model never receives the placement ID, world position or truth label.
    assert_environment()
    if not os.environ.get('ACTIVE_INSPECTION_LOCK_FD'):raise RuntimeError('standalone movement denied; authorized runner lease required')
    if os.environ.get('RANGE_CAMPAIGN_RUNNER_PID')!=str(os.getppid()):raise RuntimeError('runtime is not a direct child of the authorized runner')
    if not (args.run_dir/'reservation.json').exists():raise RuntimeError('episode not reserved in progress ledger')
    if (args.run_dir/'run_manifest.json').exists():raise FileExistsError('never replay a run manifest')
    # Check only the public lock/commit receipt. The setup plan contains world
    # coordinates and labels and is deliberately never loaded into this process.
    campaign=ROOT/'results/campaign';plan_lock=read(ROOT/'config/plan_lock.json')
    if args.run_dir.resolve()!=(campaign/'runs'/args.run_id).resolve():raise RuntimeError('runtime directory differs from reserved campaign')
    commits=[json.loads(line) for line in (campaign/'progress_events.jsonl').read_text(encoding='utf-8').splitlines()]
    matching=[e for e in commits if e['run_id']==args.run_id]
    if not matching or matching[-1]['status']!='RESERVED' or matching[-1]['phase']!=args.phase or matching[-1]['plan_sha256']!=plan_lock['plan_sha256']:raise RuntimeError('trial absent/already terminal or phase/plan changed')
    if digest(args.control)!=plan_lock['image_control_sha256']:raise RuntimeError('frozen image-control hash differs')
    receipt=read(args.run_dir/'setup_receipt.json')
    private_safety=Path(os.environ.get('ACTIVE_SAFETY_CONFIG',''))
    if receipt.get('result')!='PASS' or receipt.get('run_id')!=args.run_id or receipt.get('phase')!=args.phase or receipt.get('mode')!=args.mode:raise RuntimeError('verified setup missing or mode/phase differs')
    if private_safety.resolve()!=Path(receipt['private_safety_config']).resolve() or digest(private_safety)!=receipt['private_safety_sha256']:raise RuntimeError('private safety configuration differs from verified setup')
    if args.control.resolve()!=(ROOT/'config/image_control.json').resolve():raise RuntimeError('alternate image-control config not permitted')
    settings=read(args.control);original=__import__('yaml').safe_load((ACTIVE/'configs/control.yaml').read_text())
    if settings['capture_quality']!=original['capture_quality'] or any(settings['control'][k]!=original['control'][k] for k in ['max_alignment_moves','max_total_moves','max_wall_sec','max_joint_step_rad','min_duration_sec','joint_tolerance_rad']):raise RuntimeError('retained capture/control conditions changed')
    if args.mode=='clone':
        if args.model is None or digest(args.model)!=args.expected_model_sha256:raise RuntimeError('frozen model missing or changed')
        actor=Joint1ClonePolicy(args.model)
    elif args.mode=='rule':actor=Joint1RuleTeacher(ACTIVE/'artifacts/image_jacobian.json',gain=.5)
    else:actor=None
    mod=load_file('range_verified_motion',ACTIVE/'motion.py')
    source=Path(inspect.getfile(mod.Motion)).resolve()
    if source!=(ACTIVE/'motion.py').resolve():raise RuntimeError('wrong Motion source before ROS initialization')
    cfg=__import__('yaml').safe_load(mod.CFG.read_text())
    if cfg['ros_domain_id']!=47 or cfg['partition']!='physicalai_active_47':raise RuntimeError('wrong private safety domain/partition before ROS initialization')
    lease=load_file('range_old_lease',ACTIVE/'controller_lock.py')
    m=new_manifest(args.run_id,args.mode,args.phase,digest(args.control),actor)
    m['motion_import_source_absolute']=str(source);m['actual_environment']=assert_environment()
    write(args.run_dir/'run_manifest.json',m)
    motion=None;lock=None;owned=False
    HorizontalMotion=bind_motion(mod)
    try:
        controller_started=time.monotonic()
        lock,owned=lease.acquire()
        mod.rclpy.init();motion=HorizontalMotion()
        run_controller(motion,args.run_dir,m,settings,actor,real_observe,real_classify,started_at=controller_started)
    except Exception as e:
        m.update(result='FAULT_STOP',stage='ERROR',failure_reason=f'{type(e).__name__}: {e}',finished_at_utc=now())
        write(args.run_dir/'run_manifest.json',m)
    finally:
        if motion:motion.destroy_node()
        if mod.rclpy.ok():mod.rclpy.shutdown()
        lease.release(lock,owned)
    return 0 if m['result'] in ['PASS','QUALITY_FAIL'] else 1

if __name__=='__main__':raise SystemExit(main())
