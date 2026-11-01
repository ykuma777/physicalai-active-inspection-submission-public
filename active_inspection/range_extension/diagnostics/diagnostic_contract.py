"""Pure offline contracts for the staged, read-only runtime diagnostic."""
from __future__ import annotations

PHASE_BUDGETS_SECONDS = {
    "worker_startup": 8.0,
    "environment": 3.0,
    "imports": 20.0,
    "ros_initialization": 10.0,
    "node_and_subscriptions": 5.0,
    "discovery": 8.0,
    "controller_response": 4.0,
    "planning_scene_response": 4.0,
    "fresh_joints_and_home": 4.0,
    "ignition_discovery": 8.0,
    "target_observation": 8.0,
    "shutdown": 5.0,
}
EXPECTED_DOMAIN = "47"
EXPECTED_PARTITION = "physicalai_active_47"
REQUIRED_SERVICES = {
    "/compute_fk": "moveit_msgs/srv/GetPositionFK",
    "/check_state_validity": "moveit_msgs/srv/GetStateValidity",
    "/apply_planning_scene": "moveit_msgs/srv/ApplyPlanningScene",
    "/get_planning_scene": "moveit_msgs/srv/GetPlanningScene",
    "/plan_kinematic_path": "moveit_msgs/srv/GetMotionPlan",
    "/controller_manager/list_controllers": "controller_manager_msgs/srv/ListControllers",
}
REQUIRED_CONTROLLERS = {
    "joint_state_broadcaster", "crane_plus_arm_controller",
    "crane_plus_gripper_controller",
}


def select_controller_state_topic(candidates):
    """Prefer the controller_state endpoint; legacy state may also exist.

    Two topic names alone do not establish duplicate controller instances.
    The live gate separately checks the selected endpoint's publisher count.
    """
    preferred='/crane_plus_arm_controller/controller_state'
    legacy='/crane_plus_arm_controller/state'
    if not candidates or any(n not in (preferred,legacy) for n in candidates):
        return None
    return preferred if preferred in candidates else legacy


def environment_ok(actual: dict) -> bool:
    return (actual.get("ROS_DOMAIN_ID") == EXPECTED_DOMAIN
            and actual.get("IGN_PARTITION") == EXPECTED_PARTITION
            and actual.get("GZ_PARTITION") == EXPECTED_PARTITION)


def finite_home_ok(positions, home, tolerance: float) -> bool:
    import math
    return (isinstance(positions, (list, tuple)) and len(positions) == len(home)
            and all(math.isfinite(q) and abs(q-h) <= tolerance
                    for q, h in zip(positions, home)))


def graph_ok(types: dict, server_counts: dict, subscriber_count: int) -> bool:
    return (subscriber_count == 1
            and all(REQUIRED_SERVICES[name] in types.get(name, [])
                    and server_counts.get(name, 0) == 1
                    for name in REQUIRED_SERVICES))


def controllers_ok(states: dict) -> bool:
    return all(states.get(name) == "active" for name in REQUIRED_CONTROLLERS)


def joints_fresh(samples: list, phase_started: float, now_monotonic: float,
                 latest_clock_stamp: float | None, max_age: float = 1.0) -> bool:
    fresh = [x for x in samples if x["received_monotonic"] >= phase_started]
    if len(fresh) < 2 or latest_clock_stamp is None:
        return False
    a, b = fresh[-2:]
    return (b["header_stamp_sec"] > a["header_stamp_sec"]
            and 0 <= now_monotonic-b["received_monotonic"] <= max_age
            and -0.1 <= latest_clock_stamp-b["header_stamp_sec"] <= max_age)


def stationary_samples_ok(samples: list, phase_started: float) -> bool:
    import math
    fresh = [x for x in samples if x["received_monotonic"] >= phase_started]
    if len(fresh)<2:
        return False
    a,b=fresh[0],fresh[-1]
    velocities=b.get("velocities_rad_sec")
    return (b["received_monotonic"]-a["received_monotonic"]>=.5
            and len(a["positions_rad"])==len(b["positions_rad"])
            and max(abs(x-y) for x,y in zip(a["positions_rad"],b["positions_rad"]))<=.002
            and isinstance(velocities,list) and len(velocities)==len(b["positions_rad"])
            and all(math.isfinite(v) and abs(v)<=.01 for v in velocities))


class PhaseWatchdog:
    """Every stage gets its own measured budget; no reset on progress events."""
    def __init__(self, phase: str, started: float):
        self.phase = phase
        self.started = started

    def start(self, phase: str, at: float):
        if phase not in PHASE_BUDGETS_SECONDS:
            raise ValueError("unknown phase")
        self.phase, self.started = phase, at

    def expired(self, now: float) -> bool:
        return now-self.started > PHASE_BUDGETS_SECONDS[self.phase]
