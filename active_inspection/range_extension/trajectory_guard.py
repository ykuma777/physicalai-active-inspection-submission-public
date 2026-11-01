"""Offline-importable trajectory checks and durable, pre-send plan evidence.

The numerical limits match the former runtime guard.  No ROS imports, requests,
trajectory edits, replanning, or publications are performed by this module.
"""
from __future__ import annotations

import math
from common import now, write

JOINT_NAMES = tuple('crane_plus_joint'+str(i) for i in range(1, 5))
JOINT1_CAP_RAD = .055
JOINT1_NUMERICAL_TOLERANCE_RAD = 1e-9
OTHER_JOINT_CAP_RAD = .002


def evaluate_horizontal_path(names, start, points):
    """Check every collision-sampled execution point against this plan's start.

    The input order must be the retained configuration order. Motion.plan maps
    MoveIt's trajectory by name into this order before returning these points.
    Input-contract errors fail closed; no numerical tolerance has been widened.
    """
    limits = [JOINT1_CAP_RAD]+[OTHER_JOINT_CAP_RAD]*3
    eps = [JOINT1_NUMERICAL_TOLERANCE_RAD, 0., 0., 0.]
    result = {'result': 'PASS', 'reference': 'fresh measured start returned by Motion.plan',
              'units': 'radian', 'joint_names': list(names),
              'limits_rad': limits, 'numerical_tolerance_rad': eps,
              'point_index_base': 0, 'point_count': len(points),
              'max_abs_deviation_rad': [0.]*4, 'violations': []}
    if tuple(names) != JOINT_NAMES:
        result.update(result='FAIL', reason='configured joint name/order differs from retained horizontal contract')
        return result
    vectors = [start]+list(points)
    if not points or any(len(q) != 4 or any(isinstance(x, bool) or not math.isfinite(float(x)) for x in q) for q in vectors):
        result.update(result='FAIL', reason='empty, malformed or nonfinite planned states')
        return result
    for i, q in enumerate(points):
        for j, (value, reference) in enumerate(zip(q, start)):
            deviation = float(value)-float(reference)
            result['max_abs_deviation_rad'][j] = max(result['max_abs_deviation_rad'][j], abs(deviation))
            if abs(deviation) > limits[j]+eps[j]:
                result['violations'].append({'joint_name': names[j], 'joint_index': j,
                    'point_index': i, 'planned_absolute_rad': value,
                    'reference_absolute_rad': reference, 'signed_deviation_rad': deviation,
                    'limit_rad': limits[j], 'numerical_tolerance_rad': eps[j],
                    'reason': 'planned joint1 path exceeds per-operation cap' if j == 0 else 'planned correction moves non-horizontal joints'})
    if result['violations']:
        result.update(result='FAIL', reason=result['violations'][0]['reason'])
    return result


def json_safe(value):
    # Invalid numeric responses must still leave evidence. Such values are never
    # accepted for execution; strings below preserve their diagnostic identity.
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


class HorizontalPlanAuditMixin:
    """Wrap the unchanged Motion.plan/move through normal super() dispatch.

    Subclass supplies audit_message_to_dict (the existing ROS serializer).
    Captures requests/responses without adding any service calls or samples.
    """
    _guard_context = None
    _guard_inside_plan = False

    def joint_sample_audit(self, positions):
        latest = self.latest
        return {'observed_at_utc': now(), 'positions_rad': list(positions),
                'joint_names': list(self.names), 'joint_state': self.audit_message_to_dict(latest),
                'received_monotonic': self.received, 'sample_sequence': self.seq,
                'timestamp_semantics': 'observed_at_utc is local read time; sensor time is joint_state.header.stamp'}

    def _save_guard(self):
        context = self._guard_context
        write(context['path'], json_safe(context['record']))

    def wait_sample(self):
        q = super().wait_sample()
        if self._guard_context is not None:
            sample = self.joint_sample_audit(q)
            sample['phase'] = 'inside_Motion_plan' if self._guard_inside_plan else 'after_plan_before_publish'
            r = self._guard_context['record']
            r['joint_samples'].append(sample)
            if self._guard_inside_plan and r.get('measured_plan_start_rad') is None:
                r['measured_plan_start_rad'] = list(q)
                r['measured_plan_start_sample'] = sample
            if not self._guard_inside_plan and r.get('measured_plan_start_rad') is not None:
                r['pre_publish_measured_minus_plan_start_rad'] = [a-b for a,b in zip(q,r['measured_plan_start_rad'])]
            self._save_guard()
        return q

    def call(self, name, request, *args, **kwargs):
        capture = self._guard_context is not None and self._guard_inside_plan and name == '/plan_kinematic_path'
        if capture:
            r = self._guard_context['record']
            r['planning_request_at_utc'] = now()
            r['planning_request'] = self.audit_message_to_dict(request)
            r['requested_start_state'] = r['planning_request']['motion_plan_request']['start_state']
            self._save_guard()  # Failure to save prevents the existing request.
        response = super().call(name, request, *args, **kwargs)
        if capture:
            r['planning_response_at_utc'] = now()
            r['planning_response'] = self.audit_message_to_dict(response)
            plan_response = r['planning_response']['motion_plan_response']
            r['raw_moveit_trajectory'] = plan_response['trajectory']['joint_trajectory']
            r['returned_trajectory_start'] = plan_response.get('trajectory_start')
            # These differences are evidence only. Existing Motion checks still
            # decide acceptance; no planner tolerance is added to the step cap.
            requested = r['requested_start_state'].get('joint_state', {})
            returned = (r['returned_trajectory_start'] or {}).get('joint_state', {})
            def named_state(state):
                names, values = state.get('name', []), state.get('position', [])
                by_name = dict(zip(names, values))
                return [by_name[n] for n in self.names] if all(n in by_name for n in self.names) else None
            a, b = named_state(requested), named_state(returned)
            r['returned_start_minus_requested_start_rad'] = None if a is None or b is None else [y-x for x,y in zip(a,b)]
            self._save_guard()  # Raw names and every raw knot survive later rejection.
        return response

    def plan(self, goal):
        if self._guard_context is None:
            return super().plan(goal)
        r = self._guard_context['record']
        self._guard_inside_plan = True
        try:
            start, points, report = super().plan(goal)
            r['measured_plan_start_rad'] = list(start)
            r['execution_joint_names'] = list(self.names)
            r['all_collision_sampled_execution_points_rad'] = [list(q) for q in points]
            r['base_plan_report'] = report
            before = r.get('operation_joint_state_rad')
            if before is not None:
                r['plan_start_minus_operation_measurement_rad'] = [a-b for a,b in zip(start,before)]
            r['requested_goal_minus_plan_start_rad'] = [a-b for a,b in zip(goal,start)]
            guard = evaluate_horizontal_path(self.names, start, points)
            r['guard'] = guard
            self._save_guard()  # Persist both PASS and FAIL before returning/raising.
            if guard['result'] != 'PASS':
                raise RuntimeError(guard['reason']+'; no send/replan')
            report['horizontal_only_guard'] = guard
            r['stage'] = 'GUARD_PASS_BEFORE_SEND'
            self._save_guard()
            return start, points, report
        except BaseException as exc:
            r['stage'] = 'PLAN_REJECTED_BEFORE_SEND'
            r['plan_error'] = {'type': type(exc).__name__, 'message': str(exc), 'at_utc': now()}
            self._save_guard()
            raise
        finally:
            self._guard_inside_plan = False

    def move(self, source, target, run_dir, manifest, persist, action_record=None):
        if not target.startswith('align_'):
            return super().move(source,target,run_dir,manifest,persist,action_record=action_record)
        if self._guard_context is not None:
            raise RuntimeError('nested alignment audit denied')
        ordinal = len(manifest['motions'])+1
        path = run_dir/f'trajectory_guard_{ordinal:02d}_{target}.json'
        if path.exists():
            raise FileExistsError('trajectory diagnostic already exists; no replay')
        action = action_record or {}
        record = {'schema_version': 1, 'record_type': 'alignment_plan_evidence',
                  'evidence_origin': getattr(self,'trajectory_evidence_origin','LIVE_WHEN_AUTHORIZED'),
                  'created_at_utc': now(), 'source': source, 'target': target,
                  'stage': 'BEFORE_PLAN', 'execution_authorized_by_this_record': False,
                  'configured_joint_names': list(self.names), 'units': 'radian',
                  'operation_joint_state_rad': action.get('q_before_rad'),
                  'operation_joint_sample': action.get('operation_joint_sample'),
                  'cached_joint_state_at_move_entry': self.joint_sample_audit(
                      [dict(zip(self.latest.name,self.latest.position))[n] for n in self.names]),
                  'raw_actor_output_rad': action.get('raw_output_rad'),
                  'raw_teacher_output_rad': action.get('raw_teacher_output_rad'),
                  'raw_model_output_rad': action.get('raw_model_output_rad'),
                  'safety_limited_delta_rad': action.get('safety_limited_output_rad'),
                  'requested_delta_rad': action.get('chosen_delta_rad'),
                  'requested_absolute_goal_rad': list(self.cfg['poses'][target]),
                  'joint_samples': [],
                  'measured_plan_start_rad': None,
                  'raw_moveit_trajectory': None,
                  'all_collision_sampled_execution_points_rad': None,
                  'actual_trajectory_publish_count': 0,
                  'actual_trajectory_publish_count_exact': True,
                  'unconfirmed_send_intents': [],
                  'trajectory_publish_count_semantics': 'Confirmed publications only. exact=False with unconfirmed_send_intents means possibly sent, not proven zero. Never resend.',
                  'unchanged_tolerances': {'joint1_cap_rad': JOINT1_CAP_RAD,
                      'joint1_numeric_epsilon_rad': JOINT1_NUMERICAL_TOLERANCE_RAD,
                      'other_joint_excursion_rad': OTHER_JOINT_CAP_RAD,
                      'moveit_goal_constraint_tolerance_above_and_below_rad': .0001,
                      'legacy_planner_goal_mismatch_rad': .002,
                      'legacy_pre_publish_start_drift_rad': .01,
                      'collision_sample_step_rad': self.cfg['collision_sample_step_rad'],
                      'arrival_tolerance_rad': self.cfg['joint_tolerance_rad']}}
        self._guard_context = {'record': record, 'path': path}
        error = None
        try:
            self._save_guard()
            if action_record is not None:
                action_record['trajectory_guard_diagnostic_path'] = str(path)
                persist()
            return super().move(source,target,run_dir,manifest,persist,action_record=action_record)
        except BaseException as exc:
            error = exc
            raise
        finally:
            try:
                entries = manifest['motions'][ordinal-1:ordinal]
                count = sum(int(e.get('publish_count',0)) for e in entries)
                possible = [e.get('command_id') for e in entries if e.get('publish_intent_at_utc') and not e.get('published_at_utc') and not e.get('publish_count')]
                record.update(finished_at_utc=now(),
                    actual_trajectory_publish_count=count,
                    actual_trajectory_publish_count_exact=not possible,
                    unconfirmed_send_intents=possible,
                    confirmed_all_run_trajectory_publish_count=sum(int(e.get('publish_count',0)) for e in manifest['motions']),
                    move_result='ERROR' if error else 'RETURNED',
                    move_error=None if error is None else {'type':type(error).__name__,'message':str(error)})
                self._save_guard()
            finally:
                self._guard_context = None
