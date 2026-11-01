"""Saved synchronized records only; no ROS calls/inference/training."""
from pathlib import Path
import json,sys,hashlib,gzip,struct,bisect,math,csv
import numpy as np,cv2
ACTIVE=Path(__file__).resolve().parents[3];R=ACTIVE/'range_extension';sys.path.insert(0,str(R))
from common import read,write,digest,now
OUT=R/'results/sync_confirmation_layout_20261031_01';RD=R/'results/campaign_v3_20261030/runs/ire4_repro_layout_20261031_01'
def records(p):return [json.loads(x) for x in p.read_text().splitlines()]
def near(rows,values,t):
    j=bisect.bisect_left(values,t);j=min(range(max(0,j-1),min(len(values),j+2)),key=lambda k:abs(values[k]-t));return rows[j]
def xyz(p):return [p.get('position',{}).get(k,0) for k in ('x','y','z')]
def quat(p):return [p.get('orientation',{}).get(k,1 if k=='w' else 0) for k in ('x','y','z','w')]
def main():
    streams={n:records(RD/'continuous'/(n+'.jsonl')) for n in ['images','poses','joints','clock','gui_camera']};m=read(RD/'run_manifest.json');a=OUT/'audit';a.mkdir(exist_ok=True)
    summary={'created_at_utc':now(),'run_id':m['run_id'],'purpose':'reproduction/recording confirmation, excluded from old evaluation','task_result':m['result'],'model_call_count':m['model_call_count'],'corrections':m['alignment_trajectory_publish_count'],'all_sends':m['all_trajectory_publish_count'],'home':m['home_return'],'recording':{},'new_training':0}
    for n,rows in streams.items():
        key='received_monotonic' if n=='gui_camera' else 'sim_sec';times=[r[key] for r in rows];ds=np.diff(times);positive=ds[ds>0];span=times[-1]-times[0]
        s={'count':len(rows),'time_axis':key,'first':times[0],'last':times[-1],'span_sec':span,'effective_hz':(len(rows)-1)/span if span else None,'max_gap_sec':float(positive.max()) if len(positive) else None,'median_gap_sec':float(np.median(positive)) if len(positive) else None,'nonadvancing_stamps':int(np.sum(ds<=0))}
        if n=='images':s.update(nominal_camera_hz=10,missing_nominal_stamp_slots_estimate=int(sum(max(0,round(x/.1)-1) for x in positive)),gaps_over_0_15_sim_sec=int(np.sum(positive>.15)),source_of_loss='Cannot distinguish rendering/sensor omission, bridge/transport loss, and recorder callback loss without upstream counters; no filling/interpolation performed.')
        summary['recording'][n]=s
    poses=streams['poses'];target=np.array([xyz(p['target_world_pose'])+quat(p['target_world_pose']) for p in poses]);gui=np.array([xyz(p['message'])+quat(p['message']) for p in streams['gui_camera']]);cams=np.array([p['camera_world_from_actual_parent_matrix'] for p in poses])
    summary['target_world_xyz_quaternion_first']=target[0].tolist();summary['target_max_component_change']=np.max(np.abs(target-target[0]),axis=0).tolist();summary['gui_camera_max_component_change']=np.max(np.abs(gui-gui[0]),axis=0).tolist();summary['camera_world_translation_range_m']=np.ptp(cams[:,:3,3],axis=0).tolist();summary['camera_rotation_matrix_component_range_max']=float(np.ptp(cams[:,:3,:3],axis=0).max())
    summary['camera_pose_semantics']='Actual Gazebo model/link poses plus live-SDF fixed sensor offset; not joint FK; direct sensor world pose was NOT available.'
    joints=streams['joints'];jt=[j['sim_sec'] for j in joints];pt=[p['sim_sec'] for p in poses];sync=[]
    for im in streams['images']:
        path=RD/'continuous/camera_frames'/Path(im['path']).name;assert digest(path)==im['png_sha256'];p=near(poses,pt,im['sim_sec']);j=near(joints,jt,im['sim_sec'])
        sync.append({'image_index':im['index'],'image_sim_sec':im['sim_sec'],'received_utc':im['received_at_utc'],'png_sha256':im['png_sha256'],'pose_sim_sec':p['sim_sec'],'pose_dt_sec':p['sim_sec']-im['sim_sec'],'joint_sim_sec':j['sim_sec'],'joint_dt_sec':j['sim_sec']-im['sim_sec'],'joint_names':j['names'],'joint_positions':j['positions'],'target_world_pose':p['target_world_pose'],'camera_world_from_actual_parent_matrix':p['camera_world_from_actual_parent_matrix']})
    write(a/'IMAGE_POSE_JOINT_SYNC.json',{'rows':sync,'method':'nearest recorded sim stamps, no pose/image interpolation','max_abs_pose_dt_sec':max(abs(s['pose_dt_sec']) for s in sync),'max_abs_joint_dt_sec':max(abs(s['joint_dt_sec']) for s in sync)})
    with gzip.open(RD/'continuous/ros_camera_serialized.cdr.gz','rb') as f:
        for im in streams['images']:
            n=struct.unpack('<I',f.read(4))[0];raw=f.read(n);assert len(raw)==n and hashlib.sha256(raw).hexdigest()==im['serialized_message_sha256']
        assert f.read()==b''
    summary['raw_serialized_ROS_image_integrity']='PASS all framed CDR message hashes; no model/ROS service call in this verification'
    model_images=[]
    for obs in m['observations']:
        stamp=obs['metadata']['header_stamp'];t=stamp['sec']+stamp['nanosec']*1e-9;im=near(streams['images'],[i['sim_sec'] for i in streams['images']],t)
        model_images.append({'controller_image':obs['source_image_path'],'controller_sim_sec':t,'continuous_image_index':im['index'],'nearest_dt_sec':im['sim_sec']-t,'exact_stamp':abs(im['sim_sec']-t)<1e-6,'same_png_hash':im['png_sha256']==obs['metadata']['image_sha256']})
    summary['controller_capture_to_continuous_images']=model_images
    cmds=records(RD/'prepare_commands.jsonl');mutations=[x for x in cmds if 'service' in x['argv']]
    summary['target_mutations_logged']=len(mutations);summary['all_target_mutations_before_motion_window']=all(c['finished_at_utc']<read(RD/'MOTION_WINDOW_START.json')['at_utc'] for c in mutations)
    summary['measurement_scope']='Continuously sampled state; excludes sub-sample events and unlogged external actions. GUI pose has no sim stamp and is compared on received wall/monotonic time.'
    cap=cv2.VideoCapture(str(RD/'external_continuous.mp4'));summary['external_raw_video']={'frames':int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),'fps':cap.get(cv2.CAP_PROP_FPS),'duration_sec':cap.get(cv2.CAP_PROP_FRAME_COUNT)/cap.get(cv2.CAP_PROP_FPS)};cap.release()
    summary['model_outputs']=[{'raw':x['raw_model_output_rad'],'hard_limited':x['hard_safety_ceiling_output_rad'],'requested_delta':x['chosen_delta_rad'],'sent_endpoint':x['sent_trajectory_endpoint_rad'],'arrival':x['after_arrival']} for x in m['actions']]
    assert np.max(np.abs(target-target[0]))<1e-12 and np.max(np.abs(gui-gui[0]))<1e-12
    assert summary['all_target_mutations_before_motion_window'] and summary['target_mutations_logged']==2
    write(a/'SYNC_AUDIT.json',summary)
    # VFR lossless visual copy of every received ROS frame. Original CDR/PNGs retained.
    from datetime import datetime
    epoch=lambda s:datetime.fromisoformat(s.replace('Z','+00:00')).timestamp()
    rows=streams['images'];text='ffconcat version 1.0\n'
    for i,im in enumerate(rows):
        text+=f"file 'camera_frames/{im['index']:06d}.png'\n"
        if i+1<len(rows):text+=f"duration {epoch(rows[i+1]['received_at_utc'])-epoch(im['received_at_utc']):.6f}\n"
    (RD/'continuous/camera.ffconcat').write_text(text)
    print(json.dumps({'result':'AUDITED','target_max_change':summary['target_max_component_change'],'gui_max_change':summary['gui_camera_max_component_change'],'rates':summary['recording'],'controller_images':model_images,'external':summary['external_raw_video']}))
if __name__=='__main__':main()
