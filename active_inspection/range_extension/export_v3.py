"""Versioned v3 dataset/result export; never collects or controls."""
from pathlib import Path
import argparse,json
from common import ROOT,read,write,digest,now
from campaign_v3 import C,states
from train_policy import usable

def export(name):
    out=C/'exports'/name;out.mkdir(parents=True,exist_ok=False)
    p=read(C/'PLAN.json');s=states();episodes=[];rows=[]
    for t in p['trials']:
        e=s.get(t['run_id'])
        if e is None:continue
        rd=C/'runs'/t['run_id'];mp=rd/'run_manifest.json'
        item={**t,'status':e['status'],'manifest':str(mp),'manifest_sha256':digest(mp) if mp.exists() else None}
        if mp.exists():
            m=read(mp);item.update(model_call_count=m['model_call_count'],teacher_action_records=m['teacher_action_records'],correction_count=m['alignment_trajectory_publish_count'],all_trajectory_publish_count=m['all_trajectory_publish_count'],elapsed_sec=m.get('elapsed_sec'),home=(m.get('home_return') or {}).get('result'))
            if (rd/'score.json').exists():item['score']=read(rd/'score.json')
            if t['phase'] in ['training','tuning']:
                for i,a in enumerate(m['actions']):
                    if a.get('actor')=='rule':rows.append({**a,'run_id':t['run_id'],'placement_id':t['placement_id'],'phase':t['phase'],'action_number':i+1})
        episodes.append(item)
    valid=[row for row in rows if usable(row)]
    write(out/'episode_summary.json',{'created_at_utc':now(),'plan_sha256':digest(C/'PLAN.json'),'episodes':episodes,'training_episodes_attempted':sum(e['phase']=='training' for e in episodes),'logged_teacher_decision_records':len(rows),'valid_teacher_action_records':len(valid),'final_evaluation_placements':2,'final_comparison_runs':6})
    (out/'teacher_decisions_all.jsonl').write_text(''.join(json.dumps(x,ensure_ascii=False)+'\n' for x in rows),encoding='utf-8')
    (out/'teacher_actions.jsonl').write_text(''.join(json.dumps(x,ensure_ascii=False)+'\n' for x in valid),encoding='utf-8')
    print(json.dumps({'output':str(out),'terminal_episodes':len(episodes),'logged_teacher_decisions':len(rows),'valid_teacher_records':len(valid)}))
    return out

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--name',required=True);export(ap.parse_args().name)
