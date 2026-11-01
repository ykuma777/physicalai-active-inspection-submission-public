"""Passive continuous ROS images/joints/clock and Ignition pose recorder."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,json,sys,time,threading,subprocess,signal,gzip,struct,hashlib,os
import numpy as np,cv2
ACTIVE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ACTIVE/'range_extension'))
from common import write,now,assert_environment

def stamp(s):return int(s.get('sec',0))+int(s.get('nsec',s.get('nanosec',0)))*1e-9
def matrix(p):
    q=p.get('orientation',{});x,y,z,w=[float(q.get(k,1 if k=='w' else 0)) for k in ('x','y','z','w')]
    if abs(x*x+y*y+z*z+w*w-1)>.002:raise ValueError('bad pose quaternion')
    T=np.eye(4);T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];T[:3,3]=[p.get('position',{}).get(k,0) for k in ('x','y','z')];return T
def offset_pose(pose):
    x,y,z,r,p,yaw=map(float,pose.split());cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(yaw),np.sin(yaw)
    T=np.eye(4);T[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]];T[:3,3]=[x,y,z];return T

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seconds',type=float,default=0);a=ap.parse_args();out=a.out
    out.mkdir(parents=True,exist_ok=False);(out/'camera_frames').mkdir();assert_environment()
    capability=json.loads((ACTIVE/'range_extension/results/sync_confirmation_20261030_01/capability_probe.json').read_text());sensor=capability['sensor'];mount=offset_pose(sensor['sensor_pose'])
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image,JointState
    from rosgraph_msgs.msg import Clock
    from cv_bridge import CvBridge
    streams={n:(out/(n+'.jsonl')).open('x',buffering=1) for n in ['images','joints','clock','poses','gui_camera']}
    rawpose=gzip.open(out/'pose_raw.jsonl.gz','xb',compresslevel=1);cdr=gzip.open(out/'ros_camera_serialized.cdr.gz','xb',compresslevel=1)
    counts={n:0 for n in streams};last={n:None for n in streams};errors=[];stop=threading.Event();children=[];bridge=CvBridge();began=time.monotonic()
    def log(n,row):
        row['received_at_utc']=now();row['received_monotonic']=time.monotonic();streams[n].write(json.dumps(row)+'\n');counts[n]+=1;last[n]=time.monotonic()
    def transport(topic,name):
        err=(out/(name+'_transport.stderr.log')).open('x');p=subprocess.Popen(['ign','topic','-e','-t',topic,'--json-output'],stdout=subprocess.PIPE,stderr=err,text=True,bufsize=1);children.append((p,err));buf='';dec=json.JSONDecoder()
        try:
            for line in p.stdout:
                if stop.is_set():break
                buf+=line
                while buf.strip():
                    buf=buf.lstrip()
                    try:m,end=dec.raw_decode(buf)
                    except json.JSONDecodeError:break
                    buf=buf[end:]
                    if name=='poses':
                        rawpose.write((json.dumps({'received_at_utc':now(),'message':m})+'\n').encode())
                        by={p['name']:p for p in m.get('pose',[])};required=['active_gauge','crane_plus',sensor['parent_link']]
                        if not all(k in by for k in required):raise ValueError('pose entity missing')
                        model=matrix(by['crane_plus']);parent=matrix(by[sensor['parent_link']]);cam=model@parent@mount
                        log('poses',{'sim_sec':stamp(m.get('header',{}).get('stamp',{})),'target_world_pose':by['active_gauge'],'robot_model_world_pose':by['crane_plus'],'camera_parent_pose_in_model':by[sensor['parent_link']],
                          'camera_world_from_actual_parent_matrix':cam.tolist(),'camera_pose_kind':'COMPOSED_FROM_GAZEBO_MEASURED_MODEL_AND_LINK_PLUS_RUNTIME_FIXED_SENSOR_OFFSET_NOT_FK','direct_sensor_world_pose':None})
                    else:log('gui_camera',{'message':m,'sim_sec':stamp(m.get('header',{}).get('stamp',{}))})
        except BaseException as e:errors.append(name+':'+repr(e));stop.set()
    threads=[threading.Thread(target=transport,args=(topic,name),daemon=True) for topic,name in [('/world/active_inspection/pose/info','poses'),('/gui/camera/pose','gui_camera')]]
    for t in threads:t.start()
    node=None
    def onimage(msg):
        frame=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8');assert frame.shape==(480,640,3)
        index=counts['images'];path=out/'camera_frames'/f'{index:06d}.png';ok,png=cv2.imencode('.png',frame);assert ok;data=png.tobytes()
        path.write_bytes(data);raw=serialize_message(msg);cdr.write(struct.pack('<I',len(raw)));cdr.write(raw)
        log('images',{'index':index,'sim_sec':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'frame_id':msg.header.frame_id,'source_encoding':msg.encoding,'path':str(path),'png_sha256':hashlib.sha256(data).hexdigest(),'serialized_message_sha256':hashlib.sha256(raw).hexdigest(),'serialized_bytes':len(raw),'height':msg.height,'width':msg.width})
    def onjoint(msg):log('joints',{'sim_sec':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'names':list(msg.name),'positions':list(msg.position),'velocities':list(msg.velocity)})
    def onclock(msg):log('clock',{'sim_sec':msg.clock.sec+msg.clock.nanosec*1e-9})
    def done(signum,frame):stop.set()
    signal.signal(signal.SIGINT,done);signal.signal(signal.SIGTERM,done)
    try:
        rclpy.init();node=rclpy.create_node('sync_confirmation_passive_recorder')
        node.create_subscription(Image,'/inspection_camera/image',onimage,qos_profile_sensor_data)
        node.create_subscription(JointState,'/joint_states',onjoint,qos_profile_sensor_data)
        node.create_subscription(Clock,'/clock',onclock,qos_profile_sensor_data)
        nextwrite=0
        while not stop.is_set():
            if a.seconds and time.monotonic()-began>a.seconds:break
            rclpy.spin_once(node,timeout_sec=.01)
            if time.monotonic()>nextwrite:
                infos=node.get_publishers_info_by_topic('/inspection_camera/image');providers=[{'node':i.node_name,'namespace':i.node_namespace,'type':i.topic_type,'qos':str(i.qos_profile)} for i in infos]
                bridges=[]
                for proc in Path('/proc').iterdir():
                    if not proc.name.isdigit():continue
                    try:
                        cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode();env=(proc/'environ').read_bytes().split(b'\0')
                        if 'parameter_bridge' in cmd and '/inspection_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image' in cmd and b'ROS_DOMAIN_ID=47' in env:bridges.append({'pid':int(proc.name),'command':cmd})
                    except (OSError,PermissionError):continue
                provider_ok=len(providers)==1 and providers[0]['type']=='sensor_msgs/msg/Image' and providers[0]['node'] in ('parameter_bridge','ros_gz_bridge') and len(bridges)==1
                fresh=all(last[n] is not None and time.monotonic()-last[n]<1.5 for n in last)
                write(out/'heartbeat.json',{'at_utc':now(),'monotonic':time.monotonic(),'counts':dict(counts),'last_receive_monotonic':dict(last),'image_providers':providers,'bridge_processes':bridges,'provider_ok':provider_ok,'ready':fresh and provider_ok and all(counts[n]>=3 for n in counts),'errors':list(errors),'pid':os.getpid()})
                nextwrite=time.monotonic()+.25
    except BaseException as e:errors.append(repr(e))
    finally:
        stop.set()
        for p,err in children:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p,err in children:
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:errors.append('own transport child did not stop: '+str(p.pid))
            err.close()
        for t in threads:t.join(timeout=1)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for f in streams.values():f.close()
        rawpose.close();cdr.close();write(out/'RECORDER_RESULT.json',{'at_utc':now(),'counts':counts,'errors':errors,'result':'PASS' if not errors and all(counts[n]>=3 for n in counts) else 'STOP','camera_world_pose_directly_measured':False,'camera_parent_measured_from_Gazebo':True,'no_generated_or_interpolated_camera_frames':True})
    return 0 if not errors else 2
if __name__=='__main__':raise SystemExit(main())
