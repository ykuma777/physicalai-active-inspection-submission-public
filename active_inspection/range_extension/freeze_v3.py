"""Freeze only after the approved v3 train/tune runs, before any evaluation."""
from pathlib import Path
import argparse
from common import ROOT,ACTIVE,read,write,digest,now
from campaign_v3 import C,states
from policy import Joint1ClonePolicy
from train_policy import validate_rows
import json

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--model',type=Path,required=True);ap.add_argument('--dataset',type=Path,required=True);ap.add_argument('--summary',type=Path,required=True);a=ap.parse_args()
    if (C/'evaluation_lock.json').exists():raise FileExistsError('evaluation already frozen')
    p=read(C/'PLAN.json');s=states()
    reviewed={read(C/'LEFT_CHANGE_APPROVED_EVIDENCE.json')['reviewed_stop_run_id']} if (C/'LEFT_CHANGE_APPROVED_EVIDENCE.json').exists() and s.get('ire3_left_changed_20261030_01',{}).get('status')=='PASS' else set()
    if (C/'CAMPAIGN_STOP.json').exists() or any(e['status'] not in ['PASS','QUALITY_FAIL'] and e['run_id'] not in reviewed for e in s.values()):raise RuntimeError('campaign not qualified')
    if any(s.get(r,{}).get('status') not in ['PASS','QUALITY_FAIL'] and r not in reviewed for r in p['training_run_ids']+p['tuning_run_ids']):raise RuntimeError('collection incomplete')
    if any(r in s for r in p['evaluation_run_ids']):raise RuntimeError('evaluation already consumed')
    rows=[json.loads(x) for x in a.dataset.read_text(encoding='utf-8').splitlines() if x.strip()]
    validate_rows(rows,p,read(a.summary))
    policy=Joint1ClonePolicy(a.model);m=policy.metadata
    if m.get('offline_synthetic') or m['final_evaluation_used'] or m['training_includes_tuning']:raise RuntimeError('model provenance invalid')
    for name,path in [('dataset',a.dataset),('plan',C/'PLAN.json'),('episode_summary',a.summary)]:
        if m['inputs'][name]['sha256']!=digest(path):raise RuntimeError('fit input changed')
    if m['trainer_source_sha256']!=digest(ROOT/'train_policy.py') or m['policy_source_sha256']!=digest(ROOT/'policy.py'):raise RuntimeError('fit source changed')
    files=[Path(x['path']) for x in read(C/'COLLECTION_CONTRACT.json')['files']]
    files += [a.model,a.model.with_suffix('.json'),ROOT/'export_v3.py',ROOT/'freeze_v3.py',C/'COLLECTION_CONTRACT.json']
    write(C/'evaluation_lock.json',{'created_at_utc':now(),'model_path':str(a.model.resolve()),'model_sha256':digest(a.model),
        'final_evaluation_runs':6,'final_evaluation_placements':2,'updates_during_evaluation':False,'random_exploration':False,'teacher_fallback':False,
        'files':[{'path':str(f.resolve()),'sha256':digest(f)} for f in dict.fromkeys(files)]})
    print('EVALUATION_FROZEN_2_PLACEMENTS_6_RUNS')

if __name__=='__main__':main()
