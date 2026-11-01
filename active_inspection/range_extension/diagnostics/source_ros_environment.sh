#!/usr/bin/env bash
# STDOUT is a NUL-separated environment for the parent process only. It must
# never be written to a log because unrelated container env may contain secrets.
set -eo pipefail
for setup in /opt/ros/humble/setup.bash /home/ubuntu/airobot_ws/install/setup.bash /home/ubuntu/final_ws/install/setup.bash /home/ubuntu/camera_demo/ros2_ws/install/setup.bash; do
  printf 'SOURCE_START %s file=%s\n' "$(date -u +%FT%T.%NZ)" "$setup" >&2
  source "$setup"
  printf 'SOURCE_DONE %s file=%s\n' "$(date -u +%FT%T.%NZ)" "$setup" >&2
done
env -0
