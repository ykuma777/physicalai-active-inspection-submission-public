"""Future v2 training command, explicitly separated from the retained v1 model.

This file is prepared now; real dataset training is not performed by the
offline checks. Only allowed training rows enter fitting/normalization.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import math
import sys

import numpy as np

try:
    from .policy import (FEATURE_ORDER, OUTPUT_ORDER, MAX_STEP_RAD, features,
                         Joint1ClonePolicy, ConstantActionBaseline, sha256)
except ImportError:
    from policy import (FEATURE_ORDER, OUTPUT_ORDER, MAX_STEP_RAD, features,
                        Joint1ClonePolicy, ConstantActionBaseline, sha256)

TERMINAL = {'PASS', 'FAIL', 'INTERRUPTED', 'ABORTED', 'COMPLETED',
            'SUCCESS', 'FAILED', 'STOPPED', 'QUALITY_FAIL', 'PROTOCOL_STOP', 'FAULT_STOP'}


def phase(value):
    return {'train': 'training', 'adjustment': 'tuning', 'final': 'evaluation',
            'eval': 'evaluation'}.get(str(value), str(value))


def plan_members(plan):
    result = {p: {} for p in ['training', 'tuning', 'evaluation']}
    for trial in plan['trials']:
        ph = phase(trial['phase'])
        if ph not in result:
            continue
        run_id = trial.get('run_id', trial.get('id'))
        placement = trial.get('placement_id', trial.get('position_id'))
        if not run_id or not placement or run_id in result[ph]:
            raise ValueError('ambiguous planned run membership')
        result[ph][run_id] = placement
    v3=plan.get('schema_version')==3 and plan.get('evaluation_run_count')==6 and plan.get('evaluation_placement_count')==2
    final_count=6 if v3 else 12
    if any(len(result[p]) != n for p, n in [('training', 12), ('tuning', 2), ('evaluation', final_count)]):
        raise ValueError('plan must contain12 training,2 tuning and the explicitly authorized final-comparison count')
    groups = {p: set(values.values()) for p, values in result.items()}
    if any(groups[a] & groups[b] for a, b in [('training', 'tuning'),
                                           ('training', 'evaluation'), ('tuning', 'evaluation')]):
        raise ValueError('placement split leakage')
    ids = [set(result[p]) for p in result]
    if any(ids[i] & ids[j] for i in range(3) for j in range(i+1, 3)):
        raise ValueError('run split leakage')
    if [len(groups[p]) for p in ['training', 'tuning', 'evaluation']] != [6, 2, 2 if v3 else 4]:
        raise ValueError('placement counts differ from approved split')
    if v3:
        for pid in groups['evaluation']:
            if sorted(t['mode'] for t in plan['trials'] if t['phase']=='evaluation' and t['placement_id']==pid)!=['clone','fixed','rule']:
                raise ValueError('each final placement must have all three methods')
    return result


def usable(row):
    if row.get('actor') != 'rule':
        return False
    if not isinstance(row.get('after_arrival'), dict) or row['after_arrival'].get('result') != 'PASS':
        return False
    actual = row['after_arrival'].get('actual_positions')
    if not isinstance(actual, list) or len(actual) != 4 or not all(math.isfinite(float(v)) for v in actual):
        return False
    next_obs = row.get('next_observation')
    if not isinstance(next_obs, dict) or not next_obs.get('detection', {}).get('detected'):
        return False
    if not (next_obs.get('image_sha256') or next_obs.get('metadata', {}).get('image_sha256')):
        return False
    def stamp_ns(stamp):
        if not isinstance(stamp, dict) or 'sec' not in stamp or 'nanosec' not in stamp:
            return None
        sec, nsec = int(stamp['sec']), int(stamp['nanosec'])
        if sec < 0 or not 0 <= nsec < 1_000_000_000:
            return None
        return sec * 1_000_000_000 + nsec
    before = stamp_ns(row.get('source_image_stamp'))
    after = stamp_ns(next_obs.get('image_stamp', next_obs.get('metadata', {}).get('header_stamp')))
    if before is None or after is None or after <= before:
        return False
    return True


def distinct_magnitude_groups(values, separation=0.003):
    groups = []
    for value in sorted(abs(float(x)) for x in values):
        if not groups or value - groups[-1] >= separation:
            groups.append(value)
    return groups


def diversity_audit(rows):
    actions = [float(r['safety_limited_output_rad'][0]) for r in rows]
    groups = {side: distinct_magnitude_groups([x for x in actions if x * sign > 0])
              for side, sign in [('positive', 1), ('negative', -1)]}
    return {
        'teacher_action_records': len(rows),
        'episodes_with_valid_teacher_records': len({r['run_id'] for r in rows}),
        'placements_with_valid_teacher_records': sorted({r['placement_id'] for r in rows}),
        'joint1_action_distribution_rad': {str(k): v for k, v in sorted(Counter(round(x, 9) for x in actions).items())},
        'magnitude_groups_per_direction_rad': groups,
        'minimum_magnitude_group_separation_rad': 0.003,
        'saturated_action_count': sum(abs(x) >= MAX_STEP_RAD - 1e-9 for x in actions),
        'minimum8_is_execution_criterion_not_learning_sufficiency': True,
    }


def validate_rows(rows, plan, episode_summary):
    members = plan_members(plan)
    episodes = episode_summary.get('episodes', episode_summary.get('trials', []))
    terminal_training = {}
    for e in episodes:
        run = e.get('run_id', e.get('id'))
        if run in members['training']:
            if run in terminal_training:
                raise ValueError('duplicate episode summary')
            status = str(e.get('status', e.get('result', ''))).upper()
            if status not in TERMINAL:
                raise RuntimeError('a planned training episode is not terminal')
            terminal_training[run] = status
    if set(terminal_training) != set(members['training']):
        raise RuntimeError('all12 planned training attempts must be terminal; do not collect automatically')
    selected = {'training': [], 'tuning': []}
    seen = set()
    seen_image = set()
    for row in rows:
        run = row['run_id']
        ph = phase(row.get('phase', row.get('split', '')))
        placement = row.get('placement_id', row.get('position_id'))
        if run in members['evaluation'] or ph == 'evaluation' or placement in set(members['evaluation'].values()):
            raise ValueError('final evaluation data must not enter training/tuning dataset')
        if ph not in selected or members[ph].get(run) != placement:
            raise ValueError('row does not match frozen run and placement split')
        if row.get('actor') != 'rule':
            raise ValueError('dataset includes non-teacher action')
        action_id = row.get('action_number', row.get('step_index'))
        image_hash = row.get('source_ros_image_sha256', row.get('source_image_sha256'))
        if action_id is None and image_hash is None:
            raise ValueError('action needs action_number/step_index or source image hash')
        stamp=row.get('source_image_stamp',{})
        image_identity=(run,stamp['sec'],stamp['nanosec']) if 'sec' in stamp and 'nanosec' in stamp else (run,image_hash)
        key = (run, action_id) if action_id is not None else image_identity
        if key in seen or image_identity in seen_image:
            raise ValueError('duplicate teacher action record')
        seen.add(key)
        seen_image.add(image_identity)
        if not usable(row):
            continue
        detection = row['detection']
        joints = row['q_before_rad']
        features(detection, joints)
        delta = row['safety_limited_output_rad']
        if len(delta) != 4 or any(not math.isfinite(float(v)) for v in delta):
            raise ValueError('invalid four-joint teacher output record')
        if any(abs(float(v)) > 1e-12 for v in delta[1:]):
            raise ValueError('joint2/3/4 teacher action is outside horizontal-only scope')
        if abs(float(delta[0])) > MAX_STEP_RAD + 1e-12:
            raise ValueError('teacher action exceeds retained cap')
        # Refuse sign-inverted teacher samples relative to the retained positive Jacobian.
        ex = float(detection['center_error_xy_px'][0])
        if ex * float(delta[0]) >= 0:
            raise ValueError('teacher action does not correct horizontal error toward center')
        selected[ph].append({**row, 'placement_id': placement})
    train = selected['training']
    audit = diversity_audit(train)
    audit['planned_training_episode_count'] = 12
    audit['consumed_training_episode_count'] = len(terminal_training)
    audit['episode_terminal_status_counts'] = dict(Counter(terminal_training.values()))
    audit['dataset_rows_including_unusable'] = len(rows)
    if len(train) < 8:
        raise RuntimeError('fewer than8 valid teacher records; no automatic extra collection')
    if set(audit['placements_with_valid_teacher_records']) != set(members['training'].values()):
        raise RuntimeError('not all6 training placements have valid teacher transitions')
    if any(len(v) < 3 for v in audit['magnitude_groups_per_direction_rad'].values()):
        raise RuntimeError('teacher data lack three distinct correction magnitudes in both directions')
    if not all(any(0 < sign * float(r['safety_limited_output_rad'][0]) < MAX_STEP_RAD - 0.002
                   for r in train) for sign in [-1, 1]):
        raise RuntimeError('teacher data contain only saturated actions in a direction')
    return selected, audit


def fit_arrays(train, alpha=0.1):
    if not math.isfinite(alpha) or alpha <= 0:
        raise ValueError('ridge alpha must be positive')
    x = np.asarray([features(r['detection'], r['q_before_rad']) for r in train], dtype=np.float64)
    y = np.asarray([r['safety_limited_output_rad'][0] for r in train], dtype=np.float64)
    mean = x.mean(axis=0)
    scale = x.std(axis=0)
    scale = np.where(scale < 1e-6, 1.0, scale)
    z = np.c_[np.ones(len(x)), (x-mean)/scale]
    regularizer = np.eye(3)*alpha
    regularizer[0, 0] = 0.0
    weights = np.linalg.solve(z.T@z+regularizer, z.T@y)
    return mean, scale, weights, float(y.mean())


def error_metrics(items, mean, scale, weights, constant, command_cap=MAX_STEP_RAD):
    if not items:
        return {'records': 0, 'episodes': 0, 'model_limited_mae_rad': None,
                'constant_limited_mae_rad': None}
    xx = np.asarray([features(r['detection'], r['q_before_rad']) for r in items])
    yy = np.asarray([r['safety_limited_output_rad'][0] for r in items])
    raw = np.c_[np.ones(len(xx)), (xx-mean)/scale]@weights
    if not 0 < command_cap <= MAX_STEP_RAD:raise ValueError('requested command cap cannot exceed hard safety limit')
    pred = np.clip(raw, -command_cap, command_cap)
    base = np.clip(constant, -command_cap, command_cap)
    return {'records': len(items), 'episodes': len({r['run_id'] for r in items}),
            'model_raw_mae_rad': float(np.abs(raw-yy).mean()),
            'model_limited_mae_rad': float(np.abs(pred-yy).mean()),
            'constant_limited_mae_rad': float(np.abs(base-yy).mean()),
            'model_limited_count': int((np.abs(raw)>command_cap).sum()),
            'requested_command_cap_rad':float(command_cap),
            'model_rejected_excessive_output_count': int((np.abs(raw)>2*MAX_STEP_RAD).sum()),
            'note': 'Offline action agreement only; not a robot success-rate comparison.'}


def train(rows, plan, episode_summary, output, *, dataset_path=None,
          plan_path=None, episode_summary_path=None, alpha=None,
          alpha_candidates=(0.01, 0.1, 1.0), offline_synthetic=False):
    selected, audit = validate_rows(rows, plan, episode_summary)
    command_cap=float(plan.get('teacher',{}).get('requested_command_cap_rad',MAX_STEP_RAD))
    # All candidates are fit on training rows only. Only action agreement on
    # the separately collected tuning placements selects the fixed artifact.
    members = plan_members(plan)
    terminal = {e.get('run_id', e.get('id')): str(e.get('status', e.get('result', ''))).upper()
                for e in episode_summary.get('episodes', episode_summary.get('trials', []))}
    if any(terminal.get(run) not in TERMINAL for run in members['tuning']):
        raise RuntimeError('both planned tuning attempts must be terminal before model selection')
    if {r['placement_id'] for r in selected['tuning']} != set(members['tuning'].values()):
        raise RuntimeError('missing valid teacher actions in a tuning placement; no automatic extra collection')
    candidates = [float(alpha)] if alpha is not None else sorted(set(map(float, alpha_candidates)))
    if not candidates:
        raise ValueError('no fixed alpha candidates')
    fits=[]
    for candidate in candidates:
        fit=fit_arrays(selected['training'], candidate)
        metric=error_metrics(selected['tuning'], *fit,command_cap=command_cap)
        if metric['model_rejected_excessive_output_count']:
            continue # A rejected runtime output cannot be scored as an executed clipped action.
        score=metric['model_limited_mae_rad']
        fits.append((float(score), candidate, fit))
    if not fits:raise RuntimeError('all predeclared models reject tuning inputs; report insufficiency without extra trials')
    score, alpha, (mean, scale, weights, constant)=min(fits,key=lambda item:(item[0],item[1]))
    output = Path(output)
    metadata_path = output.with_suffix('.json')
    if output.exists() or metadata_path.exists():
        raise FileExistsError('a new model path is required; old models are immutable')
    if output.suffix != '.npz':
        raise ValueError('model path must end in .npz')
    if not offline_synthetic and not output.resolve().is_relative_to((Path(__file__).resolve().parent/'artifacts').resolve()):
        raise ValueError('real v2 model output must be under range_extension/artifacts, separate from v1')
    if offline_synthetic and 'offline' not in str(output).lower():
        raise ValueError('synthetic weights must be stored in an explicit offline folder')
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output, mean=mean, scale=scale, weights=weights)
    metrics = {p: error_metrics(items, mean, scale, weights, constant,command_cap=command_cap)
               for p, items in selected.items()}
    metadata = {
        'schema_version': 1, 'scope': 'horizontal_image_alignment_joint1_only',
        'algorithm': 'NumPy ridge regression behavior cloning',
        'trainer_source_sha256': sha256(__file__),
        'policy_source_sha256': sha256(Path(__file__).with_name('policy.py')),
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'feature_order': list(FEATURE_ORDER), 'output_order': list(OUTPUT_ORDER),
        'normalization': 'z=(x-training_mean)/training_std; nearconstant std is1',
        'intercept': 'leading constant1; intercept coefficient not regularized',
        'ridge_alpha': alpha, 'training_mean': mean.tolist(),
        'candidate_selection': {'predeclared_ridge_alphas': candidates,
                               'criterion': 'minimum safety-limited tuning teacher-action MAE; exact tie chooses smaller alpha',
                               'candidate_scores': [{'ridge_alpha': a, 'tuning_limited_action_mae_rad': s} for s,a,_ in fits],
                               'fitting_and_normalization': 'training only',
                               'evaluation_data_used': False},
        'training_scale': scale.tolist(), 'weights_intercept_then_features': weights.tolist(),
        'training_rows': len(selected['training']),
        'training_run_ids': sorted({r['run_id'] for r in selected['training']}),
        'planned_training_run_ids': list(plan_members(plan)['training']),
        'training_placements': sorted({r['placement_id'] for r in selected['training']}),
        'tuning_rows': len(selected['tuning']),
        'tuning_run_ids': sorted({r['run_id'] for r in selected['tuning']}),
        'training_includes_tuning': False, 'final_evaluation_used': False,
        'safety_max_step_rad': MAX_STEP_RAD,
        'requested_command_cap_rad':command_cap,
        'excessive_output_rejection_rad': MAX_STEP_RAD*2,
        'model_sha256': sha256(output), 'numpy_version': np.__version__,
        'teacher_diversity': audit, 'action_agreement_metrics': metrics,
        'constant_action_baseline': {'mean_training_action_rad': constant,
                                     'same_step_limit_rad': MAX_STEP_RAD,
                                     'same_requested_command_cap_rad':command_cap,
                                     'runtime_fallback': False,
                                     'tuning_model_action_mae_better_than_constant':
                                         metrics['tuning']['model_limited_mae_rad'] < metrics['tuning']['constant_limited_mae_rad'],
                                     'interpretation': 'Offline action imitation only; this is not evidence of robot success-rate superiority.'},
        'offline_synthetic': bool(offline_synthetic),
        'execution_trial_count': 0 if offline_synthetic else None,
        'inputs': {label: {'path': str(p), 'sha256': sha256(p)}
                   for label, p in [('dataset', dataset_path), ('plan', plan_path),
                                    ('episode_summary', episode_summary_path)] if p is not None},
    }
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    # Load through the real runtime path before reporting a model as prepared.
    Joint1ClonePolicy(output, allow_offline_synthetic=offline_synthetic)
    return metadata


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', type=Path, required=True)
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--episode-summary', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--ridge-alpha', type=float, default=None,
                    help='explicit fixed alpha; default selects only among predeclared0.01,0.1,1.0 using tuning action MAE')
    args = ap.parse_args()
    rows = [json.loads(s) for s in args.dataset.read_text(encoding='utf-8').splitlines() if s.strip()]
    metadata = train(rows, json.loads(args.plan.read_text(encoding='utf-8')),
                     json.loads(args.episode_summary.read_text(encoding='utf-8')),
                     args.output, dataset_path=args.dataset, plan_path=args.plan,
                     episode_summary_path=args.episode_summary, alpha=args.ridge_alpha)
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
