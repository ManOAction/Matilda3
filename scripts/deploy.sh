#!/usr/bin/env bash
# deploy.sh - Deploy the latest version of the Matilda Robot software
# DO NOT RUN WITH UNCOMMINTED CHANGES IN DEVELOPMENT! THIS WILL OVERWRITE THEM.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_DIR"

# Update code
git fetch --all --prune
git reset --hard origin/master

# Build
./scripts/build.sh

# Install Base Services (including motor controller)
if systemctl list-unit-files | grep -q '^matilda-robot.service'; then
  sudo systemctl restart matilda-robot.service
  sudo systemctl --no-pager --full status matilda-robot.service || true
else
  echo "matilda-robot.service not installed yet (that's ok)."
fi

# Install Behavior Manager
if systemctl list-unit-files | grep -q '^matilda-behavior-manager.service'; then
  sudo systemctl restart matilda-behavior-manager.service
  sudo systemctl --no-pager --full status matilda-behavior-manager.service || true
else
  echo "matilda-behavior-manager.service not installed yet (that's ok)."
fi
