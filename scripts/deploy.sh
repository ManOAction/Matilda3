#!/usr/bin/env bash
# deploy.sh - Deploy the latest version of the Matilda Robot software
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_DIR"

# Update code
git fetch --all --prune
git reset --hard origin/master

# Build
./scripts/build.sh

# Restart service if installed
if systemctl list-unit-files | grep -q '^matilda-robot.service'; then
  sudo systemctl restart matilda-robot.service
  sudo systemctl --no-pager --full status matilda-robot.service || true
else
  echo "matilda-robot.service not installed yet (that's ok)."
fi

