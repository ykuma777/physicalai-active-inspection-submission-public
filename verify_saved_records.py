"""Saved-data re-inference only. No ROS init, motion, training or publication."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import hashlib,json,ast
import numpy as np
ROOT=Path(__file__).resolve().parent
R=ROOT/'active_inspection/range_extension';C=R/'results/campaign_v3_20261030'
sys.path.insert(0,str(R))
from policy import Joint1ClonePolicy
from train_policy import usable
def load(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 model=R/'artifacts/horizontal_bc_v3_20261030.npz';meta=load(model.with_suffix('.json'))
 assert sha(model)==meta['model_sha256']=='f3a5d9f3699e7be137c0433b85e4b519593cd020ec96d218b4a350200f81d40e'
 actor=Joint1ClonePolicy(model)
 data=C/'exports/collection_v1/teacher_actions.jsonl';assert sha(data)==meta['inputs']['dataset']['sha256']
 rows=[json.loads(x) for x in data.read_text(encoding='utf-8').splitlines() if x.strip()]
 train=[r for r in rows if r['phase']=='training' and usable(r)];tune=[r for r in rows if r['phase']=='tuning' and usable(r)]
 assert len(train)==17 and len({r['run_id'] for r in train})==11 and len(tune)==4
 assert {r['run_id'] for r in train}==set(meta['training_run_ids'])
 plan=load(C/'PLAN.json');splits={p:{t['placement_id'] for t in plan['trials'] if t['phase']==p} for p in ['training','tuning','evaluation']}
 assert [len(splits[p]) for p in splits]==[6,2,2]
 assert not any(splits[a]&splits[b] for a,b in [('training','tuning'),('training','evaluation'),('tuning','evaluation')])
 evals=[];reinf=[];images=0
 for f in sorted((C/'runs').glob('*/run_manifest.json')):
  m=load(f)
  for o in m.get('observations',[]):
   p=f.parent/Path(o['source_image_path']).name;assert sha(p)==o['metadata']['image_sha256'];images+=1
  if m.get('range_extension',{}).get('phase')=='evaluation':evals.append(m)
  for i,a in enumerate(m.get('actions',[]),1):
   if a.get('actor')!='clone' or not a.get('actual_publish_count'):continue
   predicted=actor.predict_with_audit(a['detection'],a['q_before_rad'],.055)
   assert predicted['result']=='PASS'
   raw=predicted['raw_model_output_rad'];cap=[float(np.clip(predicted['safety_limited_output_rad'][0],-.0548,.0548)),0.,0.,0.]
   raw_error=max(abs(x-y) for x,y in zip(raw,a['raw_model_output_rad']))
   chosen_error=max(abs(x-y) for x,y in zip(cap,a['chosen_delta_rad']))
   assert raw_error<1e-12 and chosen_error<1e-12
   reinf.append({'run_id':m['run_id'],'operation':i,'raw_saved_data_reinference_rad':raw,'recorded_raw_rad':a['raw_model_output_rad'],'recorded_requested_delta_rad':a['chosen_delta_rad'],'max_raw_error_rad':raw_error,'max_requested_error_rad':chosen_error})
 assert len(evals)==6 and len(reinf)==6
 counts={mode:sum(m['result']=='PASS' for m in evals if m['controller']==label) for mode,label in [('fixed','fixed_observation'),('rule','image_rule_controller'),('clone','learned_image_feature_behavior_cloning')]}
 # Exact labels are preserved in the per-run source; confirm by sorted capture outcomes independent of controller names.
 outcomes=sorted(m['result'] for m in evals);assert outcomes==['PASS','PASS','PASS','PASS','QUALITY_FAIL','QUALITY_FAIL']
 parsed=0
 for p in ROOT.rglob('*.py'):
  ast.parse(p.read_text(encoding='utf-8-sig'));parsed+=1
 report={'result':'PASS','activity':'saved-data re-inference, source-image hashes, split and syntax checks only',
   'training_usable_action_records':len(train),'training_completed_episodes':11,'training_attempts':12,'tuning_records':len(tune),
   'evaluation_runs':len(evals),'evaluation_placements':2,'saved_data_reinference_count':len(reinf),'predictions':reinf,
   'checked_source_images':images,'parsed_python_files':parsed,'model_sha256':sha(model),'data_sha256':sha(data),
   'new_robot_trials':0,'new_runtime_model_calls':0,'model_refits':0,'live_ROS_reproduction':'NOT_RUN'}
 (ROOT/'OFFLINE_REPRO_QA.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({k:v for k,v in report.items() if k!='predictions'},indent=2))
if __name__=='__main__':main()
