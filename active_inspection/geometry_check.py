#!/usr/bin/env python3
"""Read live Gazebo state independently; export only known collision geometry.

The SDF response may contain visual needle state, so it is held only in this
process's memory. Neither the raw response, its digest, visuals, joint values,
truth labels nor setup records are written, printed, or returned to runtime.
"""
from __future__ import annotations
import argparse, ast, re
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parent
TOLERANCE = 0.002


def now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def command(argv, timeout=8):
    result = subprocess.run(argv, text=True, capture_output=True, timeout=timeout)
    if result.returncode:
        # Tool output can contain the raw SDF. Expose only command identity/code.
        raise RuntimeError(f'Gazebo read failed: {argv[1]}, returncode={result.returncode}')
    return result.stdout


def stringmsg_data(raw):
    # ign emits protobuf text with one C-escaped data string. A strict literal
    # matcher plus Python's safe literal parser handles its quoting, octal and
    # newline escapes without executing any content. No raw response is exposed.
    values = re.findall(r'^data:\s*("(?:[^"\\]|\\.)*")\s*$', raw, flags=re.MULTILINE)
    if len(values) != 1:
        raise ValueError('Gazebo SDF response has no unique data field')
    value = ast.literal_eval(values[0])
    if not isinstance(value,str) or not value:
        raise ValueError('Gazebo SDF response has no data')
    return value

def finite_vector(text, count, context):
    try:
        values = [float(v) for v in text.split()]
    except (TypeError, ValueError):
        raise ValueError(f'invalid {context}') from None
    if len(values) != count or any(not math.isfinite(v) for v in values):
        raise ValueError(f'invalid {context}')
    return values


def pose(element, allowed_frames, context):
    p = element.find('pose')
    if p is None:
        return [0.] * 6
    if p.get('relative_to', '') not in allowed_frames:
        raise ValueError(f'unsupported {context} reference frame')
    if p.get('degrees', 'false').lower() not in ('false', '0') or p.get('rotation_format', 'euler_rpy') != 'euler_rpy':
        raise ValueError(f'unsupported {context} orientation encoding')
    return finite_vector(p.text or '', 6, context)


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matrix(rpy):
    r,p,y = rpy
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp, cp*sr, cp*cr]]


def compose(a, b):
    # Store rotations as matrices to avoid Euler addition errors.
    axyz, arot = a
    bxyz, brot = b
    return ([axyz[i]+sum(arot[i][j]*bxyz[j] for j in range(3)) for i in range(3)], multiply(arot,brot))


def transform(p):
    return (p[:3], matrix(p[3:]))


def close_values(actual, expected, context):
    if len(actual) != len(expected) or not all(math.isfinite(v) for v in actual+expected) or any(abs(a-b) > TOLERANCE for a,b in zip(actual, expected)):
        raise ValueError(f'{context} mismatch')


def rotation_close(actual, expected, context):
    close_values([v for row in actual for v in row], [v for row in expected for v in row], context)


def safe_pose(xyz, rotation):
    return {'world_xyz': xyz, 'rotation_matrix': rotation}


def collision_box(model, model_pose, expected_link, expected_collision):
    links = model.findall('link')
    collisions = [(link,c) for link in links for c in link.findall('collision')]
    if len(collisions) != 1:
        raise ValueError('unexpected physical collision count for known static object')
    link, collision = collisions[0]
    if link.get('name') != expected_link or collision.get('name') != expected_collision:
        raise ValueError('known physical collision name mismatch')
    lp = pose(link, {'','__model__',model.get('name')}, 'link pose')
    cp = pose(collision, {'',link.get('name')}, 'collision pose')
    shape = collision.find('geometry')
    if shape is None or len(shape) != 1 or shape[0].tag != 'box':
        raise ValueError('known object physical geometry is not a single box')
    size = finite_vector(shape.findtext('box/size'), 3, 'box dimensions')
    if any(v <= 0 for v in size):
        raise ValueError('nonpositive collision box size')
    xyz, rotation = compose(compose(transform(model_pose), transform(lp)), transform(cp))
    return {'link_name':expected_link,'collision_name':expected_collision,
            'link_pose_local_xyz_rpy':lp, 'collision_pose_local_xyz_rpy':cp,
            'dimensions':size, **safe_pose(xyz,rotation)}


def actual_model_poses(world):
    raw = command(['ign','topic','-e','-t',f'/world/{world}/pose/info','-n','1','--json-output'],timeout=10)
    decoder = json.JSONDecoder()
    poses = {}
    text = raw.lstrip()
    while text:
        message,end = decoder.raw_decode(text)
        for item in message.get('pose',[]):
            p,q=item.get('position',{}),item.get('orientation',{})
            poses[item.get('name','')] = ([float(p.get(k,0.)) for k in ('x','y','z')],
                [float(q.get(k,1. if k=='w' else 0.)) for k in ('x','y','z','w')])
        text=text[end:].lstrip()
    return poses


def quaternion_matrix(q):
    if len(q)!=4 or not all(math.isfinite(v) for v in q) or abs(sum(v*v for v in q)-1.)>.002:
        raise ValueError('invalid actual model orientation')
    x,y,z,w=q
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]


def verify(config):
    world=config['world']
    if set(config['targets']) != {'G'}:
        raise ValueError('exactly one known gauge target required')
    raw = command(['ign','service','-s',f'/world/{world}/generate_world_sdf',
        '--reqtype','ignition.msgs.SdfGeneratorConfig','--reptype','ignition.msgs.StringMsg',
        '--timeout','5000','--req','global_entity_gen_config { expand_include_tags { data: true } }'],timeout=8)
    # Parse errors are replaced by a constant message so state fragments cannot
    # leak through exception text from XML/protobuf parsers.
    try:
        scene_xml=stringmsg_data(raw)
        # Fortress records topic-spawned URDF provenance as an unescaped
        # <urdf-string> URI token; escape that metadata token only.
        scene_xml=scene_xml.replace('<urdf-string>','&lt;urdf-string&gt;')
        xml=ET.fromstring(scene_xml)
        del scene_xml
    except Exception:
        raise ValueError('could not parse Gazebo world SDF safely') from None
    del raw
    sdf_world=xml.find('world')
    if sdf_world is None or sdf_world.get('name')!=world:
        raise ValueError('Gazebo world identity mismatch')
    models=sdf_world.findall('model')
    by_name={m.get('name'):m for m in models}
    if len(by_name)!=len(models):
        raise ValueError('duplicate Gazebo model identities')
    static_names={config['table']['entity']} | {v['entity'] for v in config['targets'].values()}
    allowed=static_names | {'crane_plus','camera_demo_ground'}
    if set(by_name)!=allowed:
        raise ValueError('unexpected/missing Gazebo models; collision scene incomplete')
    live=actual_model_poses(world)
    report={'world':world,'model_count':len(by_name),'source':'live Gazebo SDF + independent pose/info',
            'tolerance_m_and_rotation_component':TOLERANCE,'robot_base':{},'table':{},'targets':{}}
    safe_models={}
    for name in sorted(allowed):
        model=by_name[name]
        mp=pose(model, {'','world'}, 'model pose')
        if name not in live:
            raise ValueError('known model absent from live pose topic')
        xyz,q=live[name]
        close_values(xyz,mp[:3],'live vs SDF model position')
        rotation_close(quaternion_matrix(q),matrix(mp[3:]),'live vs SDF model orientation')
        safe_models[name]=mp
    robot_pose=safe_models['crane_plus']
    close_values(robot_pose[:3],config['robot_base_world'],'robot base position')
    rotation_close(matrix(robot_pose[3:]),matrix([0.,0.,0.]),'robot base orientation')
    report['robot_base']={'entity':'crane_plus',**safe_pose(robot_pose[:3],matrix(robot_pose[3:]))}
    table_name=config['table']['entity']
    table=by_name[table_name]
    if table.findtext('static','false').strip().lower()!='true':
        raise ValueError('table must be static')
    box=collision_box(table,safe_models[table_name],'top','collision')
    if 'model_pose_world_xyz_rpy' in config['table']:
        close_values(safe_models[table_name],config['table']['model_pose_world_xyz_rpy'],'table model pose')
    if 'top_link_pose_local_xyz_rpy' in config['table']:
        close_values(box['link_pose_local_xyz_rpy'],config['table']['top_link_pose_local_xyz_rpy'],'table top link pose')
    close_values(box['collision_pose_local_xyz_rpy'],[0.]*6,'table collision local pose')
    close_values(box['dimensions'],config['table']['dimensions'],'table size')
    close_values(box['world_xyz'],config['table']['world_center'],'table center')
    rotation_close(box['rotation_matrix'],matrix([0.,0.,0.]),'table orientation')
    report['table']={'entity':table_name,'model_pose_world_xyz_rpy':safe_models[table_name],**box}
    for target_id in ('G',):
        wanted=config['targets'][target_id]
        name=wanted['entity'];model=by_name[name];mp=safe_models[name]
        if model.findtext('static','false').strip().lower()!='true':
            raise ValueError('gauges must be static')
        close_values(mp[:3],wanted['world_bottom_xyz'],'gauge position')
        rotation_close(matrix(mp[3:]),matrix([0.,0.,wanted['yaw_rad']]),'gauge orientation')
        box=collision_box(model,mp,'body','housing_collision')
        close_values(box['dimensions'],wanted['collision_dimensions'],'gauge size')
        close_values(box['collision_pose_local_xyz_rpy'][:3],wanted['collision_center_local'],'gauge collision offset')
        rotation_close(matrix(box['collision_pose_local_xyz_rpy'][3:]),matrix([0.,0.,0.]),'gauge collision orientation')
        # Legacy body is coincident with the model; any link offset changes the
        # world collision center and must fail the independent scene contract.
        close_values(box['link_pose_local_xyz_rpy'],[0.]*6,'gauge body link pose')
        expected=compose(transform(mp),transform(wanted['collision_center_local']+[0.,0.,0.]))
        close_values(box['world_xyz'],expected[0],'gauge world collision center')
        report['targets'][target_id]={'entity':name,'model_pose_world_xyz_rpy':mp,**box}
    # Explicitly discard the full scene before returning the safe whitelist.
    del xml,sdf_world,models,by_name
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(os.environ.get('ACTIVE_SAFETY_CONFIG',str(ROOT/'generated/safety_motion.yaml'))))
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    started=time.monotonic()
    result={'schema_version':1,'started_at_utc':now(),'finished_at_utc':None,
            'result':'FAIL','error':None,'geometry':None,
            'data_contract':'known collision geometry/model poses only; no visual/needle/setup/truth data',
            'program_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'exact_argv':[sys.executable,*sys.argv]}
    try:
        cfg=yaml.safe_load(args.config.read_text(encoding='utf-8-sig'))
        result['motion_config_sha256']=hashlib.sha256(args.config.read_bytes()).hexdigest()
        if os.environ.get('ROS_DOMAIN_ID')!=str(cfg['ros_domain_id']) or any(os.environ.get(k)!=cfg['partition'] for k in ('IGN_PARTITION','GZ_PARTITION')):
            raise ValueError('wrong isolated communication domain')
        result['geometry']=verify(cfg)
        result['result']='PASS'
        status=0
    except (ValueError,RuntimeError) as exc:
        result['error']=str(exc)
        status=1
    except subprocess.TimeoutExpired:
        result['error']='Gazebo read timeout; no mutations or motion commands were sent'
        status=1
    except Exception:
        result['error']='unexpected geometry verification error; raw scene suppressed'
        status=1
    finally:
        result['finished_at_utc']=now();result['elapsed_sec']=time.monotonic()-started
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({'result':result['result'],'report':str(args.report),'elapsed_sec':result['elapsed_sec'],'error':result['error']}))
    return status

if __name__=='__main__':
    raise SystemExit(main())
