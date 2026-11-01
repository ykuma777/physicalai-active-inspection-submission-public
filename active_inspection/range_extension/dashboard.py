"""Reuse original image-hash-checked UI with explicit quality-failure labels."""
from pathlib import Path
import importlib.util,sys
from common import ACTIVE

if __name__=='__main__':
    spec=importlib.util.spec_from_file_location('range_saved_dashboard',ACTIVE/'dashboard.py')
    mod=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mod;spec.loader.exec_module(mod)
    mod.STAGES['QUALITY_FAILURE_HOME']='撮影品質不足・分類未実施・home復帰完了'
    mod.STAGES['PROTOCOL_FAILURE_HOME']='計画条件不一致・分類未実施・home復帰完了'
    mod.main()
