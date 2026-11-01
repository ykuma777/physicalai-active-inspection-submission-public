"""Read-only recorder test under the existing executor/runner leases."""
from pathlib import Path
import os,sys,subprocess,json
ACTIVE=Path(__file__).resolve().parents[1];R=ACTIVE/'range_extension';sys.path.insert(0,str(R))
from ledger import FileLease
from runner import source_environment,command
from common import read,write,assert_environment
def main():
    assert_environment();out=R/'results/sync_confirmation_20261030_01'
    if (out/'preflight_recording').exists():raise FileExistsError('no automatic repeat')
    with FileLease(R/'results/campaign/runner.lock'),FileLease(R/'results/campaign_v3_20261030/runner.lock'),FileLease('/tmp/physicalai-active-inspection-trial.lock') as lock:
        fd=lock.file.fileno();env=dict(os.environ);env['ACTIVE_INSPECTION_LOCK_FD']=str(fd);env=source_environment(out,env,fd)
        rc=command([sys.executable,str(Path(__file__).with_name('recorder.py')),'--out',str(out/'preflight_recording'),'--seconds','12'],out,'preflight_recording',env,fd,45)
        h=read(out/'preflight_recording/heartbeat.json');r=read(out/'preflight_recording/RECORDER_RESULT.json')
        write(out/'PREFLIGHT_RECORDING_GATE.json',{'result':'PASS' if rc==0 and h['ready'] and r['result']=='PASS' else 'STOP','heartbeat':h,'recorder':r,'robot_trials_consumed':0,'motion_commands':0,'target_mutations':0})
        print(json.dumps({'result':read(out/'PREFLIGHT_RECORDING_GATE.json')['result'],'heartbeat':h}));return 0 if h['ready'] else 2
if __name__=='__main__':raise SystemExit(main())
