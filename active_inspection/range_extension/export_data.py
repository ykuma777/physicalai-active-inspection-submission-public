"""Versioned dataset/summary export; only new training+tuning teacher rows."""
from pathlib import Path
import argparse,json
from common import ROOT,PLAN,read,write,now,digest
from ledger import Ledger,FileLease,TERMINAL
from runner import checked_plan,CAMPAIGN

def export(plan,ledger,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    rows=[];episodes=[];comparisons=[]
    for t in plan['trials']:
        rid=t['run_id'];entry=ledger.states.get(rid)
        if not entry or entry['status'] not in TERMINAL:continue
        episodes.append({'run_id':rid,'phase':t['phase'],'status':entry['status']})
        rd=ledger.root/'runs'/rid;mp=rd/'run_manifest.json'
        m=read(mp) if mp.exists() else {}
        if t['phase'] in ['training','tuning']:
            for i,a in enumerate(m.get('actions',[]),1):
                if a.get('actor')=='rule':rows.append({**a,'run_id':rid,'phase':t['phase'],'placement_id':t['placement_id'],'action_number':i,'source_manifest_sha256':digest(mp)})
        if t['phase']=='evaluation':
            score=read(rd/'score.json') if (rd/'score.json').exists() else {}
            comparisons.append({'run_id':rid,'placement_id':t['placement_id'],'method':t['mode'],'result':entry['status'],'capture_success':score.get('capture_success',False),'classification_correct':score.get('classification_correct'),'whole_task_success':score.get('whole_task_success',False),'all_trajectory_publish_count':m.get('all_trajectory_publish_count',0),'alignment_trajectory_publish_count':m.get('alignment_trajectory_publish_count',0),'model_call_count':m.get('model_call_count',0),'logged_controller_action_records':m.get('logged_controller_action_records',0),'teacher_action_records':m.get('teacher_action_records',0),'elapsed_sec':m.get('episode_total_elapsed_sec'),'home':(m.get('home_return') or {}).get('result'),'failure_reason':m.get('failure_reason',entry.get('reason'))})
            comparisons[-1]['all_trajectory_publish_count_exact']=m.get('all_trajectory_publish_count_exact',True)
            comparisons[-1]['unconfirmed_send_intent_command_ids']=m.get('unconfirmed_send_intent_command_ids',[])
    (output/'teacher_actions.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False,allow_nan=False)+'\n' for r in rows),encoding='utf-8')
    write(output/'episode_summary.json',{'episodes':episodes,'teacher_action_rows':len(rows),'note':'Action records are not episodes. Path probes and final data excluded from teacher dataset; failed attempts still consume slots.'})
    aggregate={}
    for mode in ['fixed','rule','clone']:
        rr=[r for r in comparisons if r['method']==mode]
        aggregate[mode]={'executed_or_failed_attempts':len(rr),'planned':4,'whole_task_successes':sum(r['whole_task_success'] for r in rr),'success_rate_on_consumed_attempts':sum(r['whole_task_success'] for r in rr)/len(rr) if rr else None,'all_trajectory_publish_counts':[r['all_trajectory_publish_count'] for r in rr],'alignment_trajectory_publish_counts':[r['alignment_trajectory_publish_count'] for r in rr]}
    write(output/'comparison_v2.json',{'plan_sha256':ledger.plan_sha256,'created_at_utc':now(),'rows':comparisons,'by_method':aggregate,'old_runs_included':False})
    write(output/'export_manifest.json',{'at_utc':now(),'plan_sha256':ledger.plan_sha256,'inputs':[{'run_id':t['run_id'],'path':str(ledger.root/'runs'/t['run_id']/'run_manifest.json'),'sha256':digest(ledger.root/'runs'/t['run_id']/'run_manifest.json')} for t in plan['trials'] if (ledger.root/'runs'/t['run_id']/'run_manifest.json').exists()],'teacher_dataset_sha256':digest(output/'teacher_actions.jsonl'),'episode_summary_sha256':digest(output/'episode_summary.json')})
    return {'teacher_records':len(rows),'terminal_episodes':len(episodes),'evaluation_attempts':len(comparisons)}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    plan=checked_plan()
    with FileLease(CAMPAIGN/'runner.lock'):
        ledger=Ledger(CAMPAIGN,plan,digest(PLAN));print(export(plan,ledger,a.output))
if __name__=='__main__':main()
