"""Finish static capture after a JSON-stream parser error; never resend GUI pose."""
from pathlib import Path
import sys,json,subprocess,os
ACTIVE=Path('/home/ubuntu/active_inspection');sys.path.insert(0,str(ACTIVE/'range_extension'))
from common import read,write,append,assert_environment,now
from ledger import FileLease
OUT=ACTIVE/'deliverables/submission_ready_20261031/work/view_A1'
def messages(raw):
 dec=json.JSONDecoder();rows=[];raw=raw.strip()
 while raw:
  r,n=dec.raw_decode(raw);rows.append(r);raw=raw[n:].strip()
 return rows
def main():
 assert_environment()
 records=[json.loads(x) for x in (OUT/'commands.jsonl').read_text().splitlines()]
 assert len([r for r in records if r['argv'][1]=='service'])==1
 last=records[-1];assert last['argv'][5:]==['-n','1','--json-output']
 poses=messages(last['stdout']);actual=poses[-1]
 config=read(ACTIVE/'deliverables/capture_layout_proposal_20261031/LAYOUT_CONFIG.json')['candidate_view_options'][0]
 xyz=[actual['position'].get(k,0) for k in ['x','y','z']];q=[actual['orientation'].get(k,0) for k in ['x','y','z','w']]
 assert max(abs(a-b) for a,b in zip(xyz,config['eye']))<.001
 assert abs(abs(sum(a*b for a,b in zip(q,config['gazebo_quaternion_xyzw'])))-1)<.0001
 write(OUT/'PARSER_TOOL_FAILURE.json',{'type':'diagnostic parser failure','reason':'ign topic -n1 returned multiple JSON objects; json.loads rejected Extra data','GUI_request_acknowledged':True,'GUI_requests_resent':0,'robot_commands':0,'task_trials':0,'resolved_offline_from_saved_response':True})
 write(OUT/'GUI_POSE.json',{'result':'PASS','actual':actual,'requested':config,'saved_messages':len(poses),'robot_commands':0,'target_mutations':0,'at_utc':now()})
 env=dict(os.environ);env.setdefault('DISPLAY',':0');env.setdefault('XAUTHORITY','/home/ubuntu/.Xauthority')
 with FileLease(ACTIVE/'range_extension/results/campaign/runner.lock'),FileLease(ACTIVE/'range_extension/results/campaign_v3_20261030/runner.lock'),FileLease('/tmp/physicalai-active-inspection-trial.lock'):
  commands=[(['ign','topic','-e','-t','/world/active_inspection/pose/info','-n','1','--json-output'],8),
   (['ffmpeg','-n','-nostdin','-f','x11grab','-video_size','1920x1080','-i',':0.0','-frames:v','1',str(OUT/'static_screen.png')],10)]
  for argv,limit in commands:
   r=subprocess.run(argv,capture_output=True,text=True,timeout=limit,env=env)
   append(OUT/'commands.jsonl',{'at_utc':now(),'argv':argv,'rc':r.returncode,'stdout':r.stdout,'stderr':r.stderr,'timeout_sec':limit})
   if r.returncode:raise RuntimeError('static read/capture failed, no repeat')
   if argv[0]=='ign':write(OUT/'world_after.json',messages(r.stdout)[-1])
 print(json.dumps({'static_capture':str(OUT/'static_screen.png'),'GUI_pose':'PASS','GUI_resend':0,'robot_commands':0,'target_mutations':0}))
if __name__=='__main__':main()
