"""One authorized GUI-camera pose change, no robot/target commands."""
from pathlib import Path
import sys,json,subprocess,time,os,argparse,math
ACTIVE=Path('/home/ubuntu/active_inspection');sys.path.insert(0,str(ACTIVE/'range_extension'))
from common import read,write,append,assert_environment,now
from ledger import FileLease
from diagnostics.direct_route import sender_processes
OUT=ACTIVE/'deliverables/submission_ready_20261031/work'
def parse_stream(raw):
 dec=json.JSONDecoder();rows=[];raw=raw.strip()
 while raw:
  obj,n=dec.raw_decode(raw);rows.append(obj);raw=raw[n:].strip()
 if not rows:raise ValueError('empty JSON stream')
 return rows
def main():
 ap=argparse.ArgumentParser();ap.add_argument('view',choices=['A1','A2']);a=ap.parse_args();assert_environment()
 dest=OUT/('view_'+a.view);dest.mkdir(exist_ok=False)
 inspection=read(OUT/'gui_inspection.json');assert 'ignition.msgs.GUICamera' in inspection['pose_service_info']
 config=read(ACTIVE/'deliverables/capture_layout_proposal_20261031/LAYOUT_CONFIG.json')
 chosen=next(v for v in config['candidate_view_options'] if v['id']==a.view);eye=chosen['eye'];q=chosen['gazebo_quaternion_xyzw']
 env=dict(os.environ);env.setdefault('DISPLAY',':0');env.setdefault('XAUTHORITY','/home/ubuntu/.Xauthority')
 def cmd(argv,timeout=8):
  r=subprocess.run(argv,capture_output=True,text=True,timeout=timeout,env=env)
  append(dest/'commands.jsonl',{'at_utc':now(),'argv':argv,'timeout_sec':timeout,'rc':r.returncode,'stdout':r.stdout,'stderr':r.stderr})
  if r.returncode:raise RuntimeError('GUI/capture request failed; no repeat')
  return r.stdout
 with FileLease(ACTIVE/'range_extension/results/campaign/runner.lock'),FileLease(ACTIVE/'range_extension/results/campaign_v3_20261030/runner.lock'),FileLease('/tmp/physicalai-active-inspection-trial.lock'):
  if sender_processes():raise RuntimeError('motion sender present')
  before=cmd(['ign','topic','-e','-t','/world/active_inspection/pose/info','-n','1','--json-output'])
  write(dest/'world_before.json',parse_stream(before)[-1])
  request=f'pose {{ position {{ x: {eye[0]} y: {eye[1]} z: {eye[2]} }} orientation {{ x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]} }} }}'
  response=cmd(['ign','service','-s','/gui/move_to/pose','--reqtype','ignition.msgs.GUICamera','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',request])
  if 'data: true' not in response:raise RuntimeError('GUI service not acknowledged; no repeat')
  time.sleep(1.5) # GUI transition settling only, not a robot motion wait.
  actual=parse_stream(cmd(['ign','topic','-e','-t','/gui/camera/pose','-n','1','--json-output']))[-1]
  actualxyz=[actual.get('position',{}).get(k,0) for k in ['x','y','z']]
  aq=[actual.get('orientation',{}).get(k,0) for k in ['x','y','z','w']]
  good=max(abs(x-y) for x,y in zip(actualxyz,eye))<.001 and abs(abs(sum(x*y for x,y in zip(aq,q)))-1)<.0001
  write(dest/'GUI_POSE.json',{'result':'PASS' if good else 'STOP','requested':chosen,'actual':actual,'at_utc':now(),'robot_commands':0,'target_mutations':0})
  if not good:raise RuntimeError('GUI pose not confirmed')
  after=cmd(['ign','topic','-e','-t','/world/active_inspection/pose/info','-n','1','--json-output']);write(dest/'world_after.json',parse_stream(after)[-1])
  cmd(['ffmpeg','-n','-nostdin','-f','x11grab','-video_size','1920x1080','-i',':0.0','-frames:v','1',str(dest/'static_screen.png')],10)
 print(json.dumps({'view':a.view,'GUI_pose':'PASS','screenshot':str(dest/'static_screen.png'),'robot_commands':0,'target_mutations':0}))
if __name__=='__main__':main()
