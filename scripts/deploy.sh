#!/usr/bin/env bash
# deploy.sh - Deploy the latest version of the Matilda Robot software
# DO NOT RUN WITH UNCOMMINTED CHANGES IN DEVELOPMENT! THIS WILL OVERWRITE THEM.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_DIR"

SYSTEMD_DIR="$REPO_DIR/systemd"

echo "[deploy.sh] Using repo dir: $REPO_DIR"
echo "[deploy.sh] Using systemd dir: $SYSTEMD_DIR"

########################################
# Update code
########################################
git fetch --all --prune
git reset --hard origin/master

########################################
# Build
########################################
./scripts/build.sh

########################################
# Install / update systemd services
########################################

install_service() {
  local name="$1"
  local src="$SYSTEMD_DIR/${name}.service"
  local dst="/etc/systemd/system/${name}.service"

  if [[ ! -f "$src" ]]; then
    echo "[deploy.sh] WARNING: $src not found, skipping ${name}.service install."
    return
  fi

  # Create symlink if it doesn't exist yet
  if [[ ! -e "$dst" ]]; then
    echo "[deploy.sh] Installing ${name}.service -> $dst"
    sudo ln -s "$src" "$dst"
  fi

  # Reload systemd to pick up any changes
  sudo systemctl daemon-reload

  # Enable (safe if already enabled)
  echo "[deploy.sh] Enabling ${name}.service"
  sudo systemctl enable "${name}.service"

  # Restart and show status
  echo "[deploy.sh] Restarting ${name}.service"
  sudo systemctl restart "${name}.service"
  sudo systemctl --no-pager --full status "${name}.service" || true
}

# Install / restart the core services
install_service "matilda-robot"
install_service "matilda-behavior-manager"
