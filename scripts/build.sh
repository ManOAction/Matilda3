#!/usr/bin/env bash
# build.sh - Build the ROS 2 workspace
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
START_TIME="$(date '+%Y-%m-%d %H:%M:%S')"

log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] [$SCRIPT_NAME] $*"
}

log "START build"

# Move to workspace directory (relative to script)
cd "$(dirname "$0")/../ws"

log "Sourcing ROS 2 Kilted environment"
set +u
source /opt/ros/kilted/setup.bash
set -u

log "Installing ROS dependencies via rosdep"
rosdep install --from-paths src --ignore-src -r -y

log "Running colcon build"
colcon build --symlink-install

log "BUILD SUCCESS"
log "Finished at $(date '+%Y-%m-%d %H:%M:%S')"
log "Total build time: $(($(date +%s) - $(date -d "$START_TIME" +%s))) seconds"