#!/usr/bin/env bash
# New diagnostic only: never launches a world, creates/removes a target or
# publishes a trajectory. Domain and partitions are read/checked, not changed.
set -eo pipefail
[[ $# -eq 1 ]] || { echo 'usage: run_readonly_once.sh UNIQUE_LABEL' >&2; exit 2; }
label=$1
[[ $label =~ ^[A-Za-z][A-Za-z0-9_-]{2,70}$ ]] || exit 2
root=/home/ubuntu/active_inspection
output=$root/range_extension/results/diagnostics/staged_readiness_$label.json
[[ ! -e $output ]] || { echo 'diagnostic output already exists; no automatic repeat' >&2; exit 2; }
printf 'SHELL_START %s label=%s domain=%s ign=%s gz=%s\n' "$(date -u +%FT%T.%NZ)" "$label" "${ROS_DOMAIN_ID-UNSET}" "${IGN_PARTITION-UNSET}" "${GZ_PARTITION-UNSET}"
for setup in /opt/ros/humble/setup.bash /home/ubuntu/airobot_ws/install/setup.bash /home/ubuntu/final_ws/install/setup.bash /home/ubuntu/camera_demo/ros2_ws/install/setup.bash; do
  printf 'SOURCE_START %s file=%s\n' "$(date -u +%FT%T.%NZ)" "$setup"
  source "$setup"
  printf 'SOURCE_DONE %s file=%s\n' "$(date -u +%FT%T.%NZ)" "$setup"
done
printf 'PYTHON_START %s domain=%s ign=%s gz=%s\n' "$(date -u +%FT%T.%NZ)" "${ROS_DOMAIN_ID-UNSET}" "${IGN_PARTITION-UNSET}" "${GZ_PARTITION-UNSET}"
exec python3 "$root/range_extension/diagnostics/staged_readiness.py" --label "$label" --output "$output"
