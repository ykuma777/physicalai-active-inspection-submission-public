"""Freeze a future real v2 model before any final comparison; no ROS calls."""
from pathlib import Path
import argparse
from common import ROOT,PLAN,read,write,digest,now
from runner import checked_plan,CAMPAIGN,check_prerequisites,saved_reviewed_stops
from ledger import Ledger,FileLease
from policy import Joint1ClonePolicy
from train_policy import validate_rows

def freeze(model,dataset,summary):
    plan=checked_plan();lock=ROOT/'artifacts/evaluation_lock.json'
    if lock.exists():raise FileExistsError('evaluation contract already frozen')
    if not model.resolve().is_relative_to((ROOT/'artifacts').resolve()):raise RuntimeError('model outside separate v2 artifacts')
    policy=Joint1ClonePolicy(model);meta=policy.metadata
    if meta.get('offline_synthetic') or meta.get('final_evaluation_used') is not False or meta.get('training_includes_tuning') is not False:raise RuntimeError('model provenance does not cover real separated training')
    rows=[__import__('json').loads(line) for line in dataset.read_text(encoding='utf-8').splitlines() if line.strip()]
    validate_rows(rows,plan,read(summary))
    for key,p in [('dataset',dataset),('plan',PLAN),('episode_summary',summary)]:
        if meta.get('inputs',{}).get(key,{}).get('sha256')!=digest(p):raise RuntimeError('model input hash mismatch: '+key)
    if meta['policy_source_sha256']!=digest(ROOT/'policy.py') or meta['trainer_source_sha256']!=digest(ROOT/'train_policy.py'):raise RuntimeError('model code changed since training')
    if meta['candidate_selection']['predeclared_ridge_alphas']!=plan['model_selection']['ridge_alphas']:raise RuntimeError('candidate model selection differs from frozen plan')
    with FileLease(CAMPAIGN/'runner.lock'):
        ledger=Ledger(CAMPAIGN,plan,digest(PLAN));check_prerequisites(plan,ledger,'evaluation')
        reviewed=saved_reviewed_stops(CAMPAIGN,digest(PLAN))
        if any(e['run_id'] not in reviewed for e in ledger.blocking()):raise RuntimeError('campaign remains stopped')
        if any(t['run_id'] in ledger.states for t in plan['trials'] if t['phase']=='evaluation'):raise RuntimeError('final evaluation already consumed; cannot retune/freeze a replacement')
        execution_files=[ROOT/name for name in ['policy.py','controller.py','runtime.py','trajectory_guard.py','runner.py','prepare_episode.py','common.py','ledger.py','dashboard.py','config/image_control.json','diagnostics/staged_readiness.py','diagnostics/diagnostic_contract.py','diagnostics/source_ros_environment.sh']]
        execution_files += [ROOT.parent/name for name in ['motion.py','geometry_check.py','detection.py']]
        execution_files += [ROOT.parent.parent/'multigauge/vision.py']
        execution_files += [ROOT.parent.parent/'camera_demo'/name for name in ['configs/runtime.yaml','artifacts/gauge_hog_svm.yml','artifacts/gauge_model_metadata.json']]
        write(lock,{'created_at_utc':now(),'model_path':str(model.resolve()),'model_sha256':digest(model),'metadata_sha256':digest(model.with_suffix('.json')),'plan_sha256':digest(PLAN),'policy_program_sha256':digest(ROOT/'policy.py'),'image_control_sha256':digest(ROOT/'config/image_control.json'),'execution_files':[{'path':str(p.resolve()),'sha256':digest(p)} for p in execution_files],'evaluation_updates':False,'random_exploration':False,'teacher_fallback':False})
    return lock

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--model',type=Path,required=True);ap.add_argument('--dataset',type=Path,required=True);ap.add_argument('--episode-summary',type=Path,required=True);a=ap.parse_args()
    print(freeze(a.model,a.dataset,a.episode_summary))
