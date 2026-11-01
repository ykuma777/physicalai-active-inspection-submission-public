#!/usr/bin/env python3
"""Bound ROS setup separately from direct diagnostic imports/init/discovery.

This optional entry point is for a bare docker exec environment. A runner that
already sourced ROS can invoke staged_readiness.py directly. No domain export
or repair is performed. Complete env stays only in memory, never in logs.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,json,os,subprocess,sys,time

HERE=Path(__file__).resolve().parent
OUT=HERE.parent/'results/diagnostics'
sys.path.insert(0,str(HERE))
from diagnostic_contract import environment_ok


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--label',required=True)
    parser.add_argument('--output')
    parser.add_argument('--recovery-state',action='store_true',help='Read-only stationary recovery assessment; home is reported, not required.')
    args=parser.parse_args()
    if not args.label.replace('_','').replace('-','').isalnum():raise ValueError('invalid label')
    OUT.mkdir(parents=True,exist_ok=True)
    progress=OUT/('source_environment_'+args.label+'.json')
    # The supervisor enforces its own exclusive output too.
    if progress.exists() or (args.output and Path(args.output).exists()):raise FileExistsError('label/output already used; no automatic repeat')
    start=time.monotonic()
    report={'created_at_utc':datetime.now(timezone.utc).isoformat(),'record_type':'bounded_ros_environment_startup',
        'budget_seconds':15,'full_environment_logged':False,'ros_initializations':0,'motion_commands_sent':0,
        'domain_or_partition_changed':False,'automatic_retries':0,
        'command':['bash',str(HERE/'source_ros_environment.sh')],
        'initial_domain_partition':{k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','IGN_PARTITION','GZ_PARTITION']}}
    env=None
    try:
        if not environment_ok(os.environ):raise RuntimeError('inherited domain/partition mismatch before sourcing')
        result=subprocess.run(report['command'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=15,check=False)
        report['returncode']=result.returncode
        report['source_stage_log']=result.stderr.decode(errors='replace').splitlines()
        if result.returncode!=0:raise RuntimeError('source command did not complete')
        env=dict(item.split('=',1) for item in result.stdout.decode(errors='strict').split('\x00') if '=' in item)
        if not environment_ok(env):raise RuntimeError('domain/partition changed or missing after sourcing')
        report['result']='PASS'
    except Exception as exc:
        report['result']='STOP';report['error']={'type':type(exc).__name__,'message':str(exc)}
        if isinstance(exc,subprocess.TimeoutExpired):
            report['source_stage_log']=(exc.stderr or b'').decode(errors='replace').splitlines()
    report['elapsed_seconds']=time.monotonic()-start
    report['finished_at_utc']=datetime.now(timezone.utc).isoformat()
    with progress.open('x',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    print(json.dumps({'source_report':str(progress),'result':report['result'],'elapsed_seconds':report['elapsed_seconds']}),flush=True)
    if report['result']!='PASS':return 2
    command=[sys.executable,str(HERE/'staged_readiness.py'),'--label',args.label]
    if args.output:command.extend(['--output',args.output])
    if args.recovery_state:command.append('--recovery-state')
    # Preserve inherited executor fd when invoked from the campaign runner.
    inherited=env.get('ACTIVE_INSPECTION_LOCK_FD')
    if inherited is not None:os.set_inheritable(int(inherited),True)
    os.execve(sys.executable,command,env)


if __name__=='__main__':raise SystemExit(main())
