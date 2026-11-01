"""Verify published saved data/models only; no ROS, training or live commands."""
from pathlib import Path
from collections import Counter
import ast,hashlib,json,sys
import numpy as np
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent
def load(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    model=ROOT/'active_inspection/range_extension/artifacts/horizontal_bc_v3_20261030.npz'
    meta=load(model.with_suffix('.json'));assert sha(model)==meta['model_sha256']
    with np.load(model) as m:
        assert m['mean'].shape==(2,) and m['scale'].shape==(2,) and m['weights'].shape==(3,)
        assert np.allclose(m['weights'],meta['weights_intercept_then_features'])
    assert meta['training_rows']==17 and len(meta['training_run_ids'])==11
    assert meta['final_evaluation_used'] is False and meta['training_includes_tuning'] is False
    cm=ROOT/'camera_demo/artifacts/gauge_hog_svm.yml'
    cmeta=load(cm.with_name('gauge_model_metadata.json'))
    assert sha(cm)==cmeta['model_sha256']
    ds=ROOT/'camera_demo/data/dataset_manifest.json';assert sha(ds)==cmeta['dataset_manifest_sha256']
    dataset=load(ds);counts=Counter()
    for row in dataset['rows']:
        assert sha(ROOT/'camera_demo'/row['relative_path'])==row['sha256']
        counts[(row['split'],row['label'])]+=1
    assert counts=={('train','normal'):12,('train','abnormal'):12,('validation','normal'):6,('validation','abnormal'):6}
    n=ROOT/'examples/normal_classification';normal=load(n/'run_manifest.json')
    assert normal['result']=='PASS' and not normal['actions']
    assert normal['classification']['label']=='normal'
    assert normal['classification']['inference']['model_sha256']==cmeta['model_sha256']
    assert sha(n/'camera_02.png')==normal['classification']['raw_camera_image_sha256']
    assert sha(n/'registered_for_svm.png')==normal['classification']['inference']['image_sha256']
    abnormal=load(ROOT/'active_inspection/range_extension/results/campaign_v3_20261030/runs/ire4_repro_layout_20261031_01/run_manifest.json')
    assert abnormal['model_call_count']==2 and abnormal['alignment_trajectory_publish_count']==2
    assert abnormal['classification']['label']=='abnormal'
    assert abnormal['classification']['inference']['model_sha256']==cmeta['model_sha256']
    assert abnormal['policy_model_sha256']==meta['model_sha256']
    pdfs=list(ROOT.rglob('*.pdf'));assert len(pdfs)==1 and pdfs[0].parent==ROOT/'docs'
    required=['active_inspection/detection.py','active_inspection/motion.py','active_inspection/range_extension/controller.py',
       'active_inspection/range_extension/trajectory_guard.py','active_inspection/range_extension/policy.py',
       'active_inspection/range_extension/train_policy.py','multigauge/vision.py','camera_demo/scripts/collect_dataset.py',
       'LICENSE','NOTICE.md','NOTICE_ORIGIN.md','THIRD_PARTY_LICENSES/CRANE_PLUS_LICENSE.txt']
    assert all((ROOT/p).is_file() for p in required)
    parsed=0
    for p in ROOT.rglob('*.py'):
        if '.git' not in p.parts:ast.parse(p.read_text(encoding='utf-8-sig'));parsed+=1
    qa={'status':'PASS','training_records':17,'completed_training_episodes':11,
        'classification_source_images':len(dataset['rows']),'classification_counts':{str(k):v for k,v in counts.items()},
        'same_classifier_for_video_examples':True,'normal_model_calls':0,'abnormal_model_calls':2,
        'bc_model_sha256':sha(model),'classifier_sha256':sha(cm),'current_pdf_sha256':sha(pdfs[0]),
        'python_sources_parsed':parsed,'new_robot_trials':0,'training_runs':0,'live_ROS_reproduction':'NOT_RUN'}
    (ROOT/'PACKAGE_QA.json').write_text(json.dumps(qa,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(qa,ensure_ascii=False))
if __name__=='__main__':main()
