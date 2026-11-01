#!/usr/bin/env python3
"""One-shot read-only diagnostic. No trajectory publisher or mutation requests.

The supervisor holds the SAME legacy executor lock. It times worker startup,
imports, ROS init, discovery and response phases independently. Only the child
created by this invocation may be terminated on a diagnostic timeout.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ACTIVE = HERE.parents[1]
OUT = ACTIVE / "range_extension/results/diagnostics"
sys.path.insert(0, str(HERE))
from diagnostic_contract import (PHASE_BUDGETS_SECONDS, EXPECTED_DOMAIN,
    EXPECTED_PARTITION, REQUIRED_SERVICES, REQUIRED_CONTROLLERS,
    PhaseWatchdog, environment_ok, finite_home_ok, graph_ok,
    controllers_ok, joints_fresh, stationary_samples_ok, select_controller_state_topic)


def utc():
    return datetime.now(timezone.utc).isoformat()


def save_new(path, value):
    with Path(path).open("x", encoding="utf-8") as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write("\n")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def worker(event_path: Path, report_path: Path, recovery_state=False) -> int:
    report = {"created_at_utc": utc(), "record_type": "staged_read_only_diagnostic",
              "motion_commands_sent": 0, "trajectory_publishers_created": 0,
              "target_mutation_requests": 0, "service_mutation_requests": 0,
              "automatic_retries": 0, "checks": {}, "measurements": {},
              "phase_budget_seconds": PHASE_BUDGETS_SECONDS,
              "service_requests_sent": [], "last_phase": "worker_startup"}
    report['recovery_state_observation']=recovery_state
    event_stream = event_path.open("a", encoding="utf-8", buffering=1)
    phase = None
    phase_start = 0.0
    node = None
    initialized = False

    def emit(kind, **extra):
        row = {"at_utc": utc(), "monotonic": time.monotonic(), "kind": kind,
               "phase": phase, **extra}
        event_stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+"\n")
        event_stream.flush()

    def start(name):
        nonlocal phase, phase_start
        phase, phase_start = name, time.monotonic()
        report["last_phase"] = name
        emit("phase_start", budget_seconds=PHASE_BUDGETS_SECONDS[name])

    def done(**extra):
        emit("phase_complete", elapsed_seconds=time.monotonic()-phase_start, **extra)

    def require(name, value, details=None):
        report["checks"][name] = bool(value)
        if not value:
            if details is not None:
                report["measurements"][name] = details
            raise RuntimeError("readiness_not_verified:"+name)

    def bounded_command(argv, seconds):
        emit("command_start", argv=argv,
             timing_scope="CLI startup, transport discovery and response are included in this named phase")
        try:
            result = subprocess.run(argv, capture_output=True, text=True,
                                    timeout=seconds, check=False)
            row = {"argv": argv, "returncode": result.returncode,
                   "stdout": result.stdout, "stderr": result.stderr,
                   "timed_out": False, "attempt_count": 1}
        except subprocess.TimeoutExpired as exc:
            row = {"argv": argv, "returncode": None, "stdout":
                   exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else exc.stdout,
                   "stderr": exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else exc.stderr,
                   "timed_out": True, "attempt_count": 1}
        emit("command_complete", **row)
        return row

    try:
        emit("worker_bootstrap", process_id=os.getpid())
        start("environment")
        import yaml
        safety_path=Path(os.environ.get("ACTIVE_SAFETY_CONFIG",str(ACTIVE/"generated/safety_motion.yaml"))).resolve()
        safety = yaml.safe_load(safety_path.read_text())
        control = yaml.safe_load((ACTIVE/"configs/control.yaml").read_text())
        envkeys = ["ROS_DOMAIN_ID", "IGN_PARTITION", "GZ_PARTITION", "RMW_IMPLEMENTATION",
                   "ROS_LOCALHOST_ONLY", "ROS_STATIC_PEERS", "IGN_IP", "GZ_IP"]
        actual = {k: os.environ.get(k) for k in envkeys}
        report["environment"] = actual
        report["expected_environment"] = {"ROS_DOMAIN_ID": EXPECTED_DOMAIN,
            "IGN_PARTITION": EXPECTED_PARTITION, "GZ_PARTITION": EXPECTED_PARTITION}
        require("domain_partition_unchanged_and_correct", environment_ok(actual))
        require("safety_domain_partition", str(safety["ros_domain_id"]) == EXPECTED_DOMAIN
                and safety["partition"] == EXPECTED_PARTITION)
        report["safety_config_absolute_path"] = str(safety_path)
        report["safety_config_sha256"] = digest(safety_path)
        report["capture_control_sha256"] = digest(ACTIVE/"configs/control.yaml")
        done(environment=actual)

        start("imports")
        sys.path.insert(0, str(ACTIVE))
        import inspect
        import motion
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from rosgraph_msgs.msg import Clock
        from controller_manager_msgs.srv import ListControllers
        from moveit_msgs.srv import GetPlanningScene
        if recovery_state:
            from control_msgs.msg import JointTrajectoryControllerState
            from action_msgs.msg import GoalStatusArray
            from direct_route import last_direct_command,sender_processes,direct_route_idle,control_subscribers
            from trajectory_msgs.msg import JointTrajectory
            history=last_direct_command()
            report['measurements']['direct_command_history']=history
            own_parent=int(os.environ['RANGE_CAMPAIGN_RUNNER_PID']) if os.environ.get('RANGE_CAMPAIGN_RUNNER_PID')==str(os.getppid()) else None
            report['measurements']['sender_processes_before']=sender_processes(own_parent)
        from controller_lock import acquire, release
        # Parent holds and passes the existing lock fd; this verifies the inode.
        lock_fd, lock_owned = acquire()
        require("existing_executor_lock_inherited", not lock_owned)
        source = str(Path(inspect.getfile(motion.Motion)).resolve())
        report["motion_import_source_absolute"] = source
        report["expected_motion_source_absolute"] = str((ACTIVE/"motion.py").resolve())
        require("motion_import_source", source == report["expected_motion_source_absolute"])
        require("motion_config_source", Path(motion.CFG).resolve() == safety_path)
        done(motion_import_source_absolute=source)

        start("ros_initialization")
        rclpy.init(args=[])
        initialized = True
        done()

        start("node_and_subscriptions")
        node = rclpy.create_node("range_extension_readonly_diagnostic_"+str(os.getpid()))
        samples = []
        clock = {"stamp": None, "received_monotonic": None}
        names = safety["joint_names"]
        def joint(msg):
            values = dict(zip(msg.name, msg.position))
            velocities=dict(zip(msg.name,msg.velocity))
            if all(name in values for name in names):
                samples.append({"received_monotonic": time.monotonic(),
                    "header_stamp_sec": float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9,
                    "positions_rad": [float(values[name]) for name in names],
                    "velocities_rad_sec":[float(velocities[name]) for name in names] if all(name in velocities for name in names) else None})
                if len(samples)>300:del samples[:-300]
        def onclock(msg):
            clock.update(stamp=float(msg.clock.sec)+float(msg.clock.nanosec)*1e-9,
                         received_monotonic=time.monotonic())
        subscriptions = [node.create_subscription(JointState,"/joint_states",joint,qos_profile_sensor_data),
                         node.create_subscription(Clock,"/clock",onclock,qos_profile_sensor_data)]
        controllers_client = node.create_client(ListControllers,"/controller_manager/list_controllers")
        planning_client = node.create_client(GetPlanningScene,"/get_planning_scene")
        command_states=[];goal_status=[];observed_commands=[]
        if recovery_state:
            subscriptions.append(node.create_subscription(JointTrajectory,'/crane_plus_arm_controller/joint_trajectory',lambda msg:observed_commands.append({'received_monotonic':time.monotonic(),'joint_names':list(msg.joint_names),'points':len(msg.points)}),10))
        done(trajectory_publisher_created=False)

        def graph_snapshot():
            types = dict(node.get_service_names_and_types())
            # Humble-compatible provider lookup: no unsupported `ros2 service info`
            # or newer count_services API is used.
            providers={name:[] for name in REQUIRED_SERVICES}
            nodes=node.get_node_names_and_namespaces()
            for node_name,node_namespace in nodes:
                for service_name,service_types in node.get_service_names_and_types_by_node(node_name,node_namespace):
                    if service_name in providers:
                        providers[service_name].append({"node_name":node_name,"node_namespace":node_namespace,"types":service_types})
            servers={name:len(rows) for name,rows in providers.items()}
            info = node.get_subscriptions_info_by_topic("/crane_plus_arm_controller/joint_trajectory")
            all_rows=[{"node_name": x.node_name,"node_namespace": x.node_namespace,"topic_type":x.topic_type} for x in info]
            rows=control_subscribers(all_rows,node.get_name(),node.get_namespace()) if recovery_state else all_rows
            return {"service_types": types, "required_service_server_counts": servers,
                    "required_service_providers":providers,
                    "arm_subscriber_count": len(rows), "arm_subscriber_nodes":rows,
                    'all_subscribers_including_passive_monitor':all_rows,"joint_callbacks_received":len(samples),
                    "latest_joint_stamp":samples[-1]["header_stamp_sec"] if samples else None,
                    "latest_clock_stamp":clock["stamp"]}

        start("discovery")
        deadline=time.monotonic()+PHASE_BUDGETS_SECONDS["discovery"]-0.2
        snap=graph_snapshot();last_logged=0;singular_since=None
        while time.monotonic()<deadline:
            snap=graph_snapshot()
            if snap["arm_subscriber_count"]>1:
                require("single_arm_subscriber",False,snap)
            if graph_ok(snap["service_types"],snap["required_service_server_counts"],snap["arm_subscriber_count"]):
                if singular_since is None:singular_since=time.monotonic()
                if time.monotonic()-singular_since>=.5:break
            else:singular_since=None
            if time.monotonic()-last_logged>=1:
                emit("discovery_progress",snapshot=snap)
                last_logged=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.1)
        report["measurements"]["discovery"]=snap
        require("required_service_types_and_exactly_one_server",all(
            REQUIRED_SERVICES[k] in snap["service_types"].get(k,[]) and snap["required_service_server_counts"][k]==1
            for k in REQUIRED_SERVICES),snap)
        require("single_arm_subscriber",snap["arm_subscriber_count"]==1,snap)
        require("single_arm_subscriber_stable_0_5sec",singular_since is not None and time.monotonic()-singular_since>=.5)
        done(snapshot=snap)

        def one_readonly_response(name,client,request,phase_name):
            start(phase_name)
            emit("readonly_service_request",service=name,attempt_count=1)
            report["service_requests_sent"].append({"service":name,"read_only":True,"count":1})
            future=client.call_async(request)
            deadline=time.monotonic()+PHASE_BUDGETS_SECONDS[phase_name]-.2
            while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
            require(phase_name+"_received",future.done() and future.result() is not None)
            done(service=name,response_received=True)
            return future.result()

        response=one_readonly_response("/controller_manager/list_controllers",controllers_client,ListControllers.Request(),"controller_response")
        states={x.name:x.state for x in response.controller}
        report["measurements"]["controller_states"]=states
        require("required_controllers_active",controllers_ok(states),states)
        if recovery_state:
            # Passive controller/action observations; never issue an action goal.
            topics=dict(node.get_topic_names_and_types())
            candidates=[name for name,types in topics.items() if name.startswith('/crane_plus_arm_controller/') and 'control_msgs/msg/JointTrajectoryControllerState' in types]
            report['measurements']['controller_state_topics']=candidates
            controller_node={(p['node_name'],p['node_namespace']) for p in snap['arm_subscriber_nodes']}
            evidence=[];valid_candidates=[]
            for name in candidates:
                pubs=node.get_publishers_info_by_topic(name)
                rows=[{'node_name':p.node_name,'node_namespace':p.node_namespace,'topic_type':p.topic_type} for p in pubs]
                valid=len(pubs)==1 and (pubs[0].node_name,pubs[0].node_namespace) in controller_node and pubs[0].topic_type=='control_msgs/msg/JointTrajectoryControllerState'
                evidence.append({'topic':name,'types':topics[name],'publishers':rows,'matches_trajectory_subscriber_controller':valid})
                if valid:valid_candidates.append(name)
            report['measurements']['controller_state_candidate_evidence']=evidence
            selected=select_controller_state_topic(valid_candidates)
            report['measurements']['selected_controller_state_topic']=selected
            report['measurements']['controller_state_selection_reason']='Expected message type and single publisher matching the arm trajectory subscriber controller; prefer controller_state over legacy state when both qualify. Joint names and freshness are checked on received data.'
            require('controller_state_topic_selected',selected is not None)
            state_publishers=node.get_publishers_info_by_topic(selected)
            report['measurements']['selected_controller_state_publishers']=[{'node_name':p.node_name,'node_namespace':p.node_namespace} for p in state_publishers]
            require('single_selected_controller_state_publisher',len(state_publishers)==1)
            def command_state(msg):
                def populated(*fields):
                    return next(((name,getattr(msg,name)) for name in fields if hasattr(msg,name) and len(getattr(msg,name).positions)>0),(None,None))
                ref_name,ref=populated('reference','desired')
                actual_name,actual=populated('feedback','actual')
                command_states.append({'received_monotonic':time.monotonic(),'stamp_sec':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                    'joint_names':list(msg.joint_names),'reference_positions':list(ref.positions) if ref else [],
                    'reference_velocities':list(ref.velocities) if ref else [],'actual_positions':list(actual.positions) if actual else [],'actual_velocities':list(actual.velocities) if actual else [],
                    'reference_field':ref_name,'actual_field':actual_name})
            def statuses(msg):
                goal_status.append({'received_monotonic':time.monotonic(),'statuses':[int(s.status) for s in msg.status_list]})
            from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
            subscriptions.append(node.create_subscription(JointTrajectoryControllerState,selected,command_state,10))
            action_topic='/crane_plus_arm_controller/follow_joint_trajectory/_action/status'
            action_providers=node.get_publishers_info_by_topic(action_topic)
            report['measurements']['action_status_providers']=[{'node_name':p.node_name,'node_namespace':p.node_namespace,'type':p.topic_type,'reliability':str(p.qos_profile.reliability),'durability':str(p.qos_profile.durability)} for p in action_providers]
            report['measurements']['action_status_requested_qos']={'reliability':'RELIABLE','durability':'TRANSIENT_LOCAL','depth':1}
            subscriptions.append(node.create_subscription(GoalStatusArray,'/crane_plus_arm_controller/follow_joint_trajectory/_action/status',statuses,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
        req=GetPlanningScene.Request();req.components.components=1
        scene=one_readonly_response("/get_planning_scene",planning_client,req,"planning_scene_response")
        report["measurements"]["planning_scene_response"]={"scene_name":scene.scene.name,"response_received":True}

        start("ignition_discovery")
        ign=bounded_command(["ign","service","-l"],PHASE_BUDGETS_SECONDS["ignition_discovery"]-.3)
        report["measurements"]["ignition_service_list"]=ign
        needed=[f"/world/{safety['world']}/create",f"/world/{safety['world']}/remove"]
        require("ignition_required_services_discovered",ign["returncode"]==0 and not ign["timed_out"]
                and all(x in (ign["stdout"] or "").splitlines() for x in needed))
        done(required_services=needed)

        start("target_observation")
        target=bounded_command(["ign","topic","-e","-t",f"/world/{safety['world']}/pose/info","-n","1","--json-output"],PHASE_BUDGETS_SECONDS["target_observation"]-.3)
        report["measurements"]["target_pose_sample"]=target
        entities=None
        if target["returncode"]==0 and not target["timed_out"]:
            try:entities=[x["name"] for x in json.loads(target["stdout"])["pose"]]
            except (KeyError,TypeError,json.JSONDecodeError):pass
        report["target_presence"]="unknown" if entities is None else "present" if "active_gauge" in entities else "absent"
        report["measurements"]["pose_entity_names"]=entities
        # This diagnostic reports presence without imposing prepare/create policy.
        require("target_presence_observed",report["target_presence"]!="unknown")
        done(target_presence=report["target_presence"])

        # Finish with fresh, advancing joints rather than an old sample cached
        # before the transport CLI phases.
        start("fresh_joints_and_home")
        fresh_start=time.monotonic();deadline=fresh_start+PHASE_BUDGETS_SECONDS["fresh_joints_and_home"]-.2
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            recent=[s for s in command_states if s['received_monotonic']>=fresh_start]
            if joints_fresh(samples,fresh_start,time.monotonic(),clock["stamp"]) and stationary_samples_ok(samples,fresh_start) and (not recovery_state or (len(recent)>=2 and recent[-1]['received_monotonic']-recent[0]['received_monotonic']>=.5)):break
        fresh=[x for x in samples if x["received_monotonic"]>=fresh_start]
        report["measurements"]["fresh_joint_samples"]=fresh
        report["measurements"]["clock"]=clock
        require("new_advancing_joint_headers_after_phase_start",joints_fresh(samples,fresh_start,time.monotonic(),clock["stamp"]))
        require("stationary_home_samples",stationary_samples_ok(samples,fresh_start))
        report["measurements"]["stationary_contract"]={"observation_span_min_sec":.5,"position_change_max_rad":.002,"velocity_abs_max_rad_sec":.01}
        home=list(map(float,control["control"]["home"]));tol=float(safety["joint_tolerance_rad"])
        report["measurements"]["home_reference_rad"]=home
        report["measurements"]["home_tolerance_rad"]=tol
        at_home=finite_home_ok(samples[-1]["positions_rad"],home,tol)
        report['at_home']=at_home
        if not recovery_state:require("fresh_joint_state_at_home",at_home)
        if recovery_state:
            import math
            publishers=node.get_publishers_info_by_topic('/crane_plus_arm_controller/joint_trajectory')
            report['measurements']['trajectory_publishers']=[{'node_name':p.node_name,'node_namespace':p.node_namespace} for p in publishers]
            report['measurements']['controller_command_states']=[s for s in command_states if s['received_monotonic']>=fresh_start]
            report['measurements']['action_goal_status']=goal_status[-1:]
            # A missing observation stays unknown rather than assuming idle.
            require('no_other_trajectory_publisher',len(publishers)==0)
            fresh_commands=[s for s in command_states if s['received_monotonic']>=fresh_start]
            require('fresh_controller_reference_received',len(fresh_commands)>=2 and fresh_commands[-1]['stamp_sec']>fresh_commands[0]['stamp_sec'] and -.1<=clock['stamp']-fresh_commands[-1]['stamp_sec']<=1.)
            last=fresh_commands[-1];ref=dict(zip(last['joint_names'],last['reference_positions']));vel=dict(zip(last['joint_names'],last['reference_velocities']));act=dict(zip(last['joint_names'],last['actual_positions']))
            require('controller_reference_joint_names',len(last['joint_names'])==len(names) and set(last['joint_names'])==set(names))
            require('controller_reference_stationary_and_reached',all(n in ref and n in act and n in vel and math.isfinite(ref[n]) and math.isfinite(act[n]) and math.isfinite(vel[n]) and abs(vel[n])<=.01 and abs(ref[n]-act[n])<=tol for n in names))
            senders=sender_processes(own_parent)
            report['measurements']['sender_processes_after']=senders
            report['measurements']['observed_topic_commands']=observed_commands
            idle=direct_route_idle(history,names,fresh_commands,clock['stamp'],senders,publishers,observed_commands,goal_status,fresh)
            report['measurements']['direct_route_idle_evidence']=idle
            require('known_direct_route_idle',idle['result']=='PASS',idle)
            require('no_sender_before_gate',not report['measurements']['sender_processes_before'])
            report['executing_command']='KNOWN_DIRECT_ROUTE_COMPLETED_NO_NEW_COMMAND_OBSERVED'
        final_infos=node.get_subscriptions_info_by_topic('/crane_plus_arm_controller/joint_trajectory')
        final_rows=[{'node_name':p.node_name,'node_namespace':p.node_namespace} for p in final_infos]
        if recovery_state:final_rows=control_subscribers(final_rows,node.get_name(),node.get_namespace())
        require("single_arm_subscriber_at_final_joint_check",len(final_rows)==1)
        done(fresh_samples=fresh[-2:])
        report["worker_result"]="PASS"
    except Exception as exc:
        report["worker_result"]="STOP"
        report["error"]={"type":type(exc).__name__,"message":str(exc),"phase":phase}
        emit("error",error=report["error"])
    finally:
        # State whether all lifecycle steps completed; parent controls final PASS.
        try:
            if initialized:
                start("shutdown")
                if node is not None:node.destroy_node()
                rclpy.shutdown()
                done()
        except Exception as exc:
            report["worker_result"]="STOP"
            report["shutdown_error"]=repr(exc)
        report["finished_at_utc"]=utc()
        save_new(report_path,report)
        emit("worker_finished",result=report.get("worker_result","STOP"))
        event_stream.close()
    return 0 if report.get("worker_result")=="PASS" else 2


def supervisor(label,output_path=None,recovery_state=False):
    if os.name!="posix":raise RuntimeError("live diagnostic is Linux/container-only; no ROS initialization")
    # No ROS imports occur in the supervisor.
    sys.path.insert(0,str(ACTIVE))
    from controller_lock import acquire,release
    fd,owned=acquire()
    stamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    OUT.mkdir(parents=True,exist_ok=True)
    prefix=OUT/("staged_readiness_"+label+"_"+stamp)
    events=Path(str(prefix)+".events.jsonl");worker_report=Path(str(prefix)+".worker.json")
    stdout=Path(str(prefix)+".stdout.log");stderr=Path(str(prefix)+".stderr.log")
    result_path=Path(output_path).resolve() if output_path else Path(str(prefix)+".json")
    if result_path.exists():
        release(fd,owned)
        raise FileExistsError("diagnostic output already exists; no ROS initialization")
    if OUT.resolve() not in result_path.parents:
        release(fd,owned)
        raise ValueError("diagnostic output must be under range_extension/results/diagnostics")
    result_path.parent.mkdir(parents=True,exist_ok=True)
    events.touch(exist_ok=False)
    env=dict(os.environ);env["ACTIVE_INSPECTION_LOCK_FD"]=str(fd)
    command=[sys.executable,str(Path(__file__).resolve()),"--worker",str(events),str(worker_report)]
    if recovery_state:command.append('--recovery-state')
    summary={"created_at_utc":utc(),"record_type":"staged_read_only_readiness_gate",
             "label":label,"command":command,"shared_executor_lock":"HELD",
             "motion_commands_sent":0,"trajectory_publishers_created":0,
             "target_mutation_requests":0,"service_mutation_requests":0,
             "automatic_retries":0,"program_sha256":digest(__file__),
             "contract_sha256":digest(HERE/"diagnostic_contract.py"),
             "phase_budget_seconds":PHASE_BUDGETS_SECONDS,
             "events_absolute_path":str(events),"worker_report_absolute_path":str(worker_report)}
    proc=None
    try:
        with stdout.open("x") as out,stderr.open("x") as err:
            started=time.monotonic()
            proc=subprocess.Popen(command,env=env,stdout=out,stderr=err,pass_fds=(fd,))
            watchdog=PhaseWatchdog("worker_startup",started)
            lines_seen=0;last_phase="worker_startup";timeout=False
            while proc.poll() is None:
                lines=events.read_text(encoding="utf-8").splitlines()
                for line in lines[lines_seen:]:
                    try:item=json.loads(line)
                    except json.JSONDecodeError:continue
                    if item.get("kind")=="phase_start":
                        watchdog.start(item["phase"],item["monotonic"]);last_phase=item["phase"]
                lines_seen=len(lines)
                if watchdog.expired(time.monotonic()):
                    timeout=True
                    summary["phase_timeout"]={"phase":last_phase,"elapsed_seconds":time.monotonic()-watchdog.started,
                                              "budget_seconds":PHASE_BUDGETS_SECONDS[last_phase]}
                    # SIGTERM applies only to our own no-motion diagnostic worker.
                    proc.terminate()
                    try:proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        summary["diagnostic_worker_still_running"]=True
                    break
                time.sleep(.05)
            summary["worker_returncode"]=proc.poll()
            summary["total_elapsed_seconds"]=time.monotonic()-started
            summary["last_phase"]=last_phase
            if worker_report.exists():
                wr=json.loads(worker_report.read_text())
                summary["checks"]=wr.get("checks",{})
                summary["worker_result"]=wr.get("worker_result")
                summary["target_presence"]=wr.get("target_presence","unknown")
                summary['at_home']=wr.get('at_home')
                summary['executing_command']=wr.get('executing_command','UNKNOWN')
            summary["gate_result"]="PASS" if not timeout and proc.poll()==0 and summary.get("worker_result")=="PASS" else "STOP"
    except Exception as exc:
        summary["gate_result"]="STOP"
        summary["supervisor_error"]={"type":type(exc).__name__,"message":str(exc)}
    finally:
        summary["finished_at_utc"]=utc()
        save_new(result_path,summary)
        release(fd,owned)
    print(json.dumps({"output":str(result_path),"gate_result":summary["gate_result"],
        "last_phase":summary.get("last_phase"),"motion_commands_sent":0,"automatic_retries":0}))
    return 0 if summary["gate_result"]=="PASS" else 2


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--label",default="range_extension_once")
    parser.add_argument("--output",help="Exclusive new summary JSON under range_extension/results/diagnostics")
    parser.add_argument("--worker",nargs=2,metavar=("EVENT_PATH","REPORT_PATH"))
    parser.add_argument('--recovery-state',action='store_true')
    args=parser.parse_args()
    if args.worker:return worker(*map(Path,args.worker),recovery_state=args.recovery_state)
    if not args.label.replace("_","").replace("-","").isalnum():raise ValueError("invalid label")
    return supervisor(args.label,args.output,args.recovery_state)


if __name__=="__main__":raise SystemExit(main())
