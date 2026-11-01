"""Setup boundary: known coordinates never enter a policy argument.

This file has no standalone execution entrypoint. The authorized runner owns
the old shared executor lease before calling prepare_episode.
"""
from __future__ import annotations
from pathlib import Path
import importlib.util,math,sys,time
from common import ACTIVE,REPO,ROOT,now,read,write,append,digest,assert_environment

def load_setup_tools():
    # The historical target generator and one-shot transport helpers are reused.
    spec=importlib.util.spec_from_file_location('range_legacy_setup',REPO/'multigauge/prepare.py')
    mod=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mod;spec.loader.exec_module(mod)
    return mod

def build_safety(plan,placement):
    import yaml
    base=yaml.safe_load((REPO/'multigauge/configs/motion.yaml').read_text(encoding='utf-8'))
    xyz=placement['world_bottom_xyz']
    base.update(world='active_inspection',ros_domain_id=47,partition='physicalai_active_47')
    base['targets']={'G':{'entity':'active_gauge','world_bottom_xyz':xyz,'yaw_rad':0.,'collision_dimensions':[.0775,.07,.12],'collision_center_local':[-.00375,0.,.06]}}
    base['poses']={'home':plan['initial_home'],'scan_center':plan['initial_observation']}
    base['motion_min_duration_sec']=6.;base['joint_tolerance_rad']=.02
    return base

def prepare_episode(plan,trial,rd,campaign_root):
    import yaml
    assert_environment()
    if 'ACTIVE_INSPECTION_LOCK_FD' not in __import__('os').environ:raise RuntimeError('setup requires inherited executor lease')
    tools=load_setup_tools();p=plan['placements'][trial['placement_id']]
    generated=rd/'private_setup';generated.mkdir(exist_ok=False)
    safety=build_safety(plan,p)
    safety_path=generated/'safety_motion.yaml'
    safety_path.write_text(yaml.safe_dump(safety,sort_keys=False),encoding='utf-8')
    truth=rd/'setup_truth.json';log=rd/'prepare_commands.jsonl'
    record={'run_id':trial['run_id'],'placement_id':trial['placement_id'],'phase':trial['phase'],
            'started_at_utc':now(),'result':'PREPARING','entity':'active_gauge','truth_label':p['needle_label'],
            'needle_angle_rad':p['needle_angle_rad'],'true_world_bottom_xyz':p['world_bottom_xyz'],
            'service_mutation_requests':0,'automatic_retries':0,'safety_config':str(safety_path)}
    write(truth,record)
    owner_path=Path(campaign_root)/'target_ownership.json'
    try:
        actual=tools.entity_poses(safety['world'],log) # one initial presence observation
        if 'active_gauge' in actual:
            if not owner_path.exists():raise RuntimeError('existing target not owned by this campaign; no remove/create sent')
            owner=read(owner_path)
            if owner.get('entity')!='active_gauge' or owner.get('verified') is not True:raise RuntimeError('target ownership unknown; stopped')
            if max(abs(x-y) for x,y in zip(actual['active_gauge']['xyz'],owner['actual_world_xyz']))>.002:raise RuntimeError('present target differs from verified ownership; stopped')
            record['service_mutation_requests']+=1;write(truth,record)
            tools.mutation(safety['world'],'remove','ignition.msgs.Entity','name: "active_gauge" type: MODEL',log)
            remaining=tools.entity_poses(safety['world'],log) # one confirmation, no polling/retry
            if 'active_gauge' in remaining:raise RuntimeError('remove acknowledged but target still present; no repeat')
        # Absence is confirmed. There is no unconditional remove request.
        sdf=generated/'target.sdf';sdf.write_text(tools.make_sdf('active_gauge',p['needle_angle_rad'],safety['targets']['G']),encoding='utf-8')
        xyz=p['world_bottom_xyz']
        request=f'name: "active_gauge" sdf_filename: "{sdf}" allow_renaming: false pose {{ position {{ x: {xyz[0]} y: {xyz[1]} z: {xyz[2]} }} orientation {{ w: 1 }} }}'
        record['service_mutation_requests']+=1;write(truth,record)
        tools.mutation(safety['world'],'create','ignition.msgs.EntityFactory',request,log)
        actual=tools.entity_poses(safety['world'],log) # one fresh created-pose confirmation
        if 'active_gauge' not in actual:raise RuntimeError('created target not observed; no resend')
        if max(abs(x-y) for x,y in zip(actual['active_gauge']['xyz'],xyz))>.002:raise RuntimeError('created target pose mismatch')
        record.update(result='PASS',finished_at_utc=now(),actual_target_pose=actual['active_gauge']);write(truth,record)
        write(owner_path,{'entity':'active_gauge','verified':True,'run_id':trial['run_id'],'actual_world_xyz':actual['active_gauge']['xyz'],'at_utc':now()})
        write(rd/'setup_receipt.json',{'result':'PASS','run_id':trial['run_id'],'phase':trial['phase'],'mode':trial['mode'],'private_safety_config':str(safety_path.resolve()),'private_safety_sha256':digest(safety_path),'truth_or_position_included':False})
        return safety_path
    except BaseException as e:
        record.update(result='FAULT_STOP',finished_at_utc=now(),error=f'{type(e).__name__}: {e}',target_state_after_failure='UNKNOWN_UNLESS_SAVED_OBSERVATION_PROVES_IT')
        write(truth,record)
        # Even a timed-out remove might have completed. Never resend/remove here.
        raise
