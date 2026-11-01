"""Horizontal controller adapter. Importing this module never initializes ROS.

run_controller has injected motion/capture/classification for offline contracts.
Real ROS binding is in runtime.py and requires the authorized runner lease.
"""
from __future__ import annotations
from pathlib import Path
import time
from common import ROOT,now,write,digest,finite_vector
from policy import Joint1RuleTeacher,Joint1ClonePolicy

class QualityFailure(Exception):pass
class ProtocolFailure(Exception):pass

def publication_ownership_ok(publishers,node_name,node_namespace):
    return len(publishers)==1 and publishers[0]['node_name']==node_name and publishers[0]['node_namespace']==node_namespace

def stable(a,b,limits):
    if not a['detection']['detected'] or not b['detection']['detected']:return False
    ma,mb=a['metadata'],b['metadata']
    sa,sb=ma['header_stamp'],mb['header_stamp']
    if (sb['sec'],sb['nanosec'])<=(sa['sec'],sa['nanosec']) or mb['received_at_utc']<=ma['received_at_utc']:return False
    return max(abs(x-y) for x,y in zip(a['detection']['center_xy'],b['detection']['center_xy']))<=limits['max_center_change_px'] and abs(a['detection']['dial_width_px']-b['detection']['dial_width_px'])<=limits['max_width_change_px']

def new_manifest(run_id,mode,phase,control_sha,policy=None):
    return {'schema_version':2,'run_id':run_id,'started_at_utc':now(),'result':'RUNNING','stage':'STARTING',
       'controller':{'fixed':'fixed_observation','rule':'rule_based_image_feedback','clone':'learned_image_feature_behavior_cloning'}[mode],
       'range_extension':{'axis':'horizontal','learned_output_joints':[1],'phase':phase,'teacher_fallback':False},
       'config_sha256':control_sha,'policy_model_sha256':getattr(policy,'model_sha256',None),
       'motions':[],'observations':[],'actions':[],'policy_inferences':[],
       'model_call_count':0,'logged_controller_action_records':0,'valid_rule_action_records':0,'teacher_action_records':0,
       'classification':None,'home_return':None,'failure_reason':None,'source_model_sha256':None,
       'all_trajectory_publish_count':0,'alignment_trajectory_publish_count':0,'initial_correction_required':None}

def run_controller(motion,rd,m,settings,actor,observe,classify,*,clock=time.monotonic,sleep=time.sleep,started_at=None):
    rd=Path(rd);limits=settings['capture_quality'];controls=settings['control'];start=clock() if started_at is None else started_at
    current='home';last=None
    def persist():
        m['logged_controller_action_records']=len(m['actions'])
        m['all_trajectory_publish_count']=sum(int(x.get('publish_count',0)) for x in m['motions'])
        m['alignment_trajectory_publish_count']=sum(int(x.get('actual_publish_count',0)) for x in m['actions'])
        uncertain=[x['command_id'] for x in m['motions'] if x.get('publish_intent_at_utc') and not x.get('published_at_utc') and not x.get('publish_count')]
        m['unconfirmed_send_intent_command_ids']=uncertain
        m['all_trajectory_publish_count_exact']=not uncertain
        m['trajectory_publish_count_semantics']='Confirmed publications only; an unconfirmed publish intent means possibly sent, not proven zero. Never resend.'
        write(rd/'run_manifest.json',m)
    def capture(after):
        sleep(.35)
        obs=observe(motion,rd,len(m['observations'])+1,after,limits)
        # Callback already verifies image SHA, arrival-time and ROS timestamp.
        if m['observations']:
            old=m['observations'][-1]['metadata'];new=obs['metadata']
            a,b=old['header_stamp'],new['header_stamp']
            if (b['sec'],b['nanosec'])<=(a['sec'],a['nanosec']):raise RuntimeError('new capture stamp did not advance; no repeated action')
        m['observations'].append(obs);persist();return obs
    def go_home():
        nonlocal current
        if len(m['motions'])>=controls['max_total_moves']:raise RuntimeError('no home trajectory budget')
        m['stage']='RETURN_HOME';persist()
        motion.move(current,'home',rd,m,persist)
        current='home';m['home_return']=motion.confirm_home();persist()
    status='FAULT_STOP'
    try:
        motion.register_scene(rd/'collision_scene.json')
        m['initial_home']=motion.confirm_home()
        m['controller_readiness']=motion.wait_controller_ready();persist()
        m['stage']='SEARCHING';persist()
        entry=motion.move('home','scan_center',rd,m,persist);current='scan_center'
        last=capture(entry['arrival'])
        found=last['detection']
        if not found['detected']:raise ProtocolFailure('initial gauge not detected; horizontal design not met')
        ex,ey=found['center_error_xy_px'];width=found['dial_width_px']
        m['initial_correction_required']=abs(ex)>limits['max_center_error_px'];persist()
        if not m['initial_correction_required']:raise ProtocolFailure('initial image already meets horizontal criterion; no motion for appearance')
        if abs(ey)>limits['max_center_error_px'] or not limits['quality_width_px'][0]<=width<=limits['quality_width_px'][1]:raise ProtocolFailure('initial vertical/size condition cannot be corrected by joint1')
        corrections=0
        while True:
            if clock()-start>controls['max_wall_sec']:raise TimeoutError('existing300s controller wall limit exceeded')
            if last['quality']['result']=='PASS':
                second=capture(m['motions'][-1]['arrival'])
                if second['quality']['result']=='PASS' and stable(last,second,limits):last=second;break
                last=second
                if last['quality']['result']=='PASS':raise QualityFailure('new images did not satisfy stability condition')
            if actor is None:raise QualityFailure('fixed observation did not meet capture quality; classification not performed')
            if corrections>=controls['max_alignment_moves']:raise QualityFailure('unchanged alignment-move budget exhausted')
            if len(m['motions'])>=controls['max_total_moves']-1:raise QualityFailure('motion budget reserves home return')
            if not last['detection']['detected']:raise QualityFailure('dial lost; no blind correction')
            # Do not start an extra correction without leaving time for normal home.
            if controls['max_wall_sec']-(clock()-start)<2*float(motion.cfg['motion_wall_timeout_sec'])+1:raise QualityFailure('time budget reserved for current correction and home')
            q=finite_vector(motion.wait_sample(),4)
            sample_audit=motion.joint_sample_audit(q) if hasattr(motion,'joint_sample_audit') else {'positions_rad':q[:],'observed_at_utc':now(),'sensor_timestamp':'UNAVAILABLE_IN_INJECTED_ADAPTER'}
            inferred={'source_image_sha256':last['metadata']['image_sha256'],'at_utc':now(),'result':'CALL_STARTED'}
            if m['controller']=='learned_image_feature_behavior_cloning':
                m['model_call_count']+=1;m['policy_inferences'].append(inferred);persist()
            audit=actor.predict_with_audit(last['detection'],q,controls['max_joint_step_rad'])
            if m['controller']=='learned_image_feature_behavior_cloning':inferred.update(audit);persist()
            if audit['result']!='PASS':raise RuntimeError('actor output guard rejected; no teacher fallback')
            delta=finite_vector(audit['safety_limited_output_rad'],4)
            if any(x!=0 for x in delta[1:]) or abs(delta[0])>.055:raise RuntimeError('horizontal joint1-only cap violated')
            if abs(delta[0])<.002:raise QualityFailure('actor step too small while capture fails; no fallback')
            action={'source_ros_image_sha256':last['metadata']['image_sha256'],'source_image_path':last['source_image_path'],
              'source_image_stamp':last['metadata']['header_stamp'],'detection':last['detection'],'q_before_rad':q,'operation_joint_sample':sample_audit,**audit,
              'chosen_delta_rad':delta,'requested_joint_goal_rad':[a+b for a,b in zip(q,delta)],
              'validated_plan_joint_goal_rad':None,'sent_trajectory_endpoint_rad':None,'sent_command_id':None,
              'actual_sent_at_utc':None,'actual_publish_count':0,'after_arrival':None,'next_observation':None}
            m['actions'].append(action);persist()
            name=f'align_{corrections+1:02d}';motion.cfg['poses'][name]=action['requested_joint_goal_rad']
            m['stage']='ALIGNING';persist()
            entry=motion.move(current,name,rd,m,persist,action_record=action);current=name
            action['after_arrival']=entry['arrival'];persist();corrections+=1
            last=capture(entry['arrival'])
            action['next_observation']={'image_sha256':last['metadata']['image_sha256'],'image_stamp':last['metadata']['header_stamp'],'received_at_utc':last['metadata']['received_at_utc'],'detection':last['detection'],'quality':last['quality']}
            if audit['actor']=='rule' and entry['arrival']['result']=='PASS':
                m['valid_rule_action_records']+=1
                if m['range_extension']['phase'] in ['training','tuning'] and last['detection']['detected']:m['teacher_action_records']+=1
            persist()
        m['stage']='CAPTURE_COMPLETE';persist()
        m['stage']='AI_RUNNING';persist()
        m['classification']=classify(last,rd)
        m['source_model_sha256']=m['classification']['inference']['model_sha256']
        m['stage']='AI_RESULT';persist();sleep(float(controls['result_hold_sec']))
        go_home();status='PASS';m['stage']='COMPLETE_HOME'
    except (QualityFailure,ProtocolFailure) as e:
        m['failure_reason']=f'{type(e).__name__}: {e}';persist()
        try:
            go_home();status='PROTOCOL_STOP' if isinstance(e,ProtocolFailure) else 'QUALITY_FAIL'
            m['stage']='PROTOCOL_FAILURE_HOME' if isinstance(e,ProtocolFailure) else 'QUALITY_FAILURE_HOME'
        except Exception as home_error:
            m['failure_reason']+=f'; HOME_FAULT: {type(home_error).__name__}: {home_error}'
            status='FAULT_STOP';m['stage']='ERROR'
    except Exception as e:
        m['failure_reason']=f'{type(e).__name__}: {e}';status='FAULT_STOP';m['stage']='ERROR'
        if m['policy_inferences'] and m['policy_inferences'][-1].get('result')=='CALL_STARTED':
            m['policy_inferences'][-1].update(result='ERROR',error=m['failure_reason'])
        # Unknown communication/planning/arrival state: no automatic home/send.
    finally:
        m['result']=status;m['finished_at_utc']=now();m['elapsed_sec']=clock()-start
        m['path_arrival_image_home_pass']=bool(m['home_return'] and m['home_return'].get('result')=='PASS' and m['observations'] and all(x.get('arrival',{}).get('result')=='PASS' for x in m['motions']))
        persist()
    return m
