"""Evidence for the controlled direct-topic route; no ROS imports or calls."""
from pathlib import Path
import json,os,math

ACTIVE=Path(__file__).resolve().parents[2]

def control_subscribers(rows,own_name,own_namespace):
    """Exclude only this diagnostic's explicitly created passive monitor."""
    return [r for r in rows if (r['node_name'],r['node_namespace'])!=(own_name,own_namespace)]


def sender_processes(allowed_parent=None):
    """Read only /proc, scoped to known user-level trajectory executors."""
    patterns=('active_inspection/control.py','active_inspection/run_trial.sh',
              'range_extension/runtime.py','range_extension/runner.py',
              'range_extension/campaign_v3.py','range_extension/live_v3.py',
              'multigauge/coordinator.py','multigauge/control.py')
    found=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) in (os.getpid(),allowed_parent):continue
        try:
            args=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            env=(p/'environ').read_bytes().split(b'\0')
            if b'ROS_DOMAIN_ID=47' not in env:continue
            if any(x in args for x in patterns):
                found.append({'pid':int(p.name),'command':args,'stat':(p/'stat').read_text()})
        except (OSError,PermissionError):continue
    return found


def last_direct_command():
    roots=[ACTIVE/'results/runs',ACTIVE/'range_extension/results/campaign/runs',ACTIVE/'range_extension/results/campaign_v3_20261030/runs']
    sends=[];intents=[]
    for root in roots:
        if not root.exists():continue
        for path in root.glob('*/run_manifest.json'):
            m=json.loads(path.read_text(encoding='utf-8'))
            for e in m.get('motions',[]):
                if e.get('publish_intent_at_utc') and not e.get('published_at_utc') and not e.get('publish_count'):
                    intents.append({'path':str(path),'time':e['publish_intent_at_utc'],'command_id':e.get('command_id')})
                if e.get('publish_count')==1 and e.get('published_at_utc'):
                    sends.append({'manifest':str(path),'run_id':m.get('run_id'),'entry':e})
    if not sends:return {'result':'UNKNOWN','reason':'no confirmed direct publication history'}
    last=max(sends,key=lambda x:x['entry']['published_at_utc']);e=last['entry']
    ambiguous=[i for i in intents if i['time']>=e['published_at_utc']]
    trajectory=e.get('trajectory') or {};arrival=e.get('arrival') or {}
    points=trajectory.get('points') or []
    good=bool(points) and arrival.get('result')=='PASS' and not ambiguous
    stamp=trajectory.get('header',{}).get('stamp',{})
    good=good and stamp.get('sec')==0 and stamp.get('nanosec')==0
    arrival_stamp=arrival.get('joint_state_header',{}).get('stamp',{})
    if 'sec' not in arrival_stamp:good=False
    return {'result':'VERIFIED_COMPLETED_DIRECT_COMMAND' if good else 'UNKNOWN',
            **last,'unconfirmed_intents_after_last_send':ambiguous,
            'joint_names':trajectory.get('joint_names'),
            'endpoint_rad':points[-1]['positions'] if points else None,
            'arrival_sim_sec':arrival_stamp.get('sec',0)+arrival_stamp.get('nanosec',0)*1e-9,
            'end_time_from_start':points[-1].get('time_from_start') if points else None,
            'semantics':'Motion uses immediate header stamp zero, one publication, and arrival only after all trajectory time plus stopped settling. This history is not an action-status inference.'}


def continuous_stationary(commands,names,joint_samples=None):
    """Position-only controller feedback has no velocity array by design.

    Actual velocities then come from the named joint_state_broadcaster stream,
    not from a fabricated zero or an empty-array assumption.
    """
    joints=joint_samples or []
    joint_velocity_proof=len(joints)>=2 and joints[-1]['received_monotonic']-joints[0]['received_monotonic']>=.5 and all(
        len(s.get('velocities_rad_sec') or [])==len(names)
        and all(math.isfinite(v) and abs(v)<=.01 for v in s['velocities_rad_sec'])
        and max(abs(a-b) for a,b in zip(s['positions_rad'],joints[0]['positions_rad']))<=.002 for s in joints)
    return bool(commands) and all(
        len(r.get('actual_positions',[]))==len(names)
        and len(r.get('reference_velocities',[]))==len(names)
        and all(math.isfinite(v) and abs(v)<=.01 for v in r['reference_velocities'])
        and ((len(r.get('actual_velocities',[]))==len(names) and all(math.isfinite(v) and abs(v)<=.01 for v in r['actual_velocities'])) or (not r.get('actual_velocities') and joint_velocity_proof))
        and max(abs(a-b) for a,b in zip(r['actual_positions'],commands[0]['actual_positions']))<=.002
        and max(abs(a-b) for a,b in zip(r['actual_positions'],r['reference_positions']))<=.02
        for r in commands)


def direct_route_idle(history,names,commands,clock,sender_rows,publishers,observed_messages,statuses,joint_samples=None):
    """No single stationary sample or absent action status establishes idle.

    This applies to the isolated, logged Motion direct-topic executors. It does
    not certify arbitrary unlogged external clients or promise future inactivity.
    """
    checks={'last_direct_command_complete':history.get('result')=='VERIFIED_COMPLETED_DIRECT_COMMAND',
            'no_sender_process':not sender_rows,'no_trajectory_publisher':not publishers,
            'no_new_topic_command_during_gate':not observed_messages}
    expected=dict(zip(history.get('joint_names') or [],history.get('endpoint_rad') or []))
    checks['clock_after_confirmed_trajectory_end']=clock is not None and clock>history.get('arrival_sim_sec',float('inf'))
    checks['reference_observation_span']=len(commands)>=2 and commands[-1]['received_monotonic']-commands[0]['received_monotonic']>=.5
    matches=True
    for row in commands:
        ref=dict(zip(row['joint_names'],row['reference_positions']))
        matches=matches and len(row['joint_names'])==len(names) and set(row['joint_names'])==set(names)
        matches=matches and all(n in expected and n in ref and math.isfinite(ref[n]) and abs(ref[n]-expected[n])<=1e-9 for n in names)
    checks['controller_reference_is_known_completed_endpoint']=bool(commands) and matches
    checks['continuous_controller_stationary']=continuous_stationary(commands,names,joint_samples)
    codes=statuses[-1]['statuses'] if statuses else None
    checks['no_observed_active_or_unknown_action_goal']=codes is None or all(s in (4,5,6) for s in codes)
    return {'result':'PASS' if all(checks.values()) else 'STOP','checks':checks,
            'action_status_observation':'NOT_RECEIVED_NOT_A_DIRECT_TOPIC_ACK' if codes is None else 'EMPTY_GOAL_LIST' if not codes else 'TERMINAL_GOALS_ONLY' if checks['no_observed_active_or_unknown_action_goal'] else 'ACTIVE_OR_UNKNOWN_GOAL',
            'applicability':'Known logged direct-topic Motion route under exclusive lock; action status alone is neither an acknowledgement nor an idle proof.'}
