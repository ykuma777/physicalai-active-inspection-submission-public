"""One read-only recording-capability probe. No ROS init or mutations."""
from pathlib import Path
import json,sys,subprocess,hashlib,datetime,xml.etree.ElementTree as ET
ACTIVE=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ACTIVE));sys.path.insert(0,str(ACTIVE/'range_extension'))
from geometry_check import stringmsg_data
from common import write,append,assert_environment,now
from ledger import FileLease

def main():
    out=ACTIVE/'range_extension/results/sync_confirmation_layout_20261031_01';out.mkdir(parents=True,exist_ok=True)
    assert_environment()
    if (out/'capability_probe.json').exists():raise FileExistsError('never automatically repeat capability probe')
    report={'started_at_utc':now(),'ROS_requests':0,'mutations':0,'motion_commands':0,'result':'STOP'}
    def cmd(argv,timeout):
        r=subprocess.run(argv,text=True,capture_output=True,timeout=timeout)
        append(out/'probe_commands.jsonl',{'at_utc':now(),'argv':argv,'timeout_sec':timeout,'returncode':r.returncode,'stderr':r.stderr})
        if r.returncode:raise RuntimeError('read failed; no retry')
        return r.stdout
    try:
        with FileLease('/tmp/physicalai-active-inspection-trial.lock'):
            raw=cmd(['ign','service','-s','/world/active_inspection/generate_world_sdf','--reqtype','ignition.msgs.SdfGeneratorConfig','--reptype','ignition.msgs.StringMsg','--timeout','5000','--req','global_entity_gen_config { expand_include_tags { data: true } }'],8)
            sdf=stringmsg_data(raw).replace('<urdf-string>','&lt;urdf-string&gt;');(out/'runtime_world_before.sdf').write_text(sdf)
            world=ET.fromstring(sdf).find('world');robot=world.find("model[@name='crane_plus']");target=world.find("model[@name='active_gauge']")
            sensors=[]
            for link in robot.findall('link'):
                for sensor in link.findall('sensor'):
                    if sensor.findtext('topic')=='/inspection_camera/image':
                        sensors.append({'robot':'crane_plus','parent_link':link.get('name'),'sensor':sensor.get('name'),'type':sensor.get('type'),'sensor_pose':sensor.findtext('pose','0 0 0 0 0 0'),'sensor_pose_attributes':sensor.find('pose').attrib if sensor.find('pose') is not None else {},'link_pose':link.findtext('pose','0 0 0 0 0 0'),'sensor_xml':ET.tostring(sensor,encoding='unicode')})
            if len(sensors)!=1:raise RuntimeError('one live camera sensor not confirmed')
            report['sensor']=sensors[0];report['target_static']=target.findtext('static') if target is not None else None
            poses=cmd(['ign','topic','-e','-t','/world/active_inspection/pose/info','-n','3','--json-output'],10)
            (out/'pose_probe_raw.jsonstream').write_text(poses)
            msgs=[];text=poses.lstrip();dec=json.JSONDecoder()
            while text:
                m,n=dec.raw_decode(text);msgs.append(m);text=text[n:].lstrip()
            wanted={'crane_plus','active_gauge',sensors[0]['parent_link']}
            if not all(wanted.issubset({p['name'] for p in m['pose']}) for m in msgs):raise RuntimeError('live target/robot/camera-parent pose not all available')
            report['pose_messages']=len(msgs);report['pose_names']=[p['name'] for p in msgs[-1]['pose']];report['pose_sim_stamps']=[m.get('header',{}).get('stamp') for m in msgs]
            report['camera_state_semantics']='Gazebo parent link pose is measured from pose/info; camera world pose may be composed from actual model/link poses plus fixed runtime-SDF sensor offset. This is not joint-angle FK and not a directly published sensor world pose.'
            report['result']='POSE_AND_SENSOR_CAPABILITY_PASS_IMAGE_PROVIDER_PENDING'
    except BaseException as e:report['error']=f'{type(e).__name__}: {e}'
    report['finished_at_utc']=now();write(out/'capability_probe.json',report)
    print(json.dumps(report));return 0 if report['result'].startswith('POSE_AND') else 2
if __name__=='__main__':raise SystemExit(main())
