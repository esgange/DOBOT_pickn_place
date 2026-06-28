#!/usr/bin/env bash
set -Eeuo pipefail

# One-click remote-readiness bootstrap for a fresh Ubuntu 22.04 robot/node PC.
# Target assumption: Ubuntu 22.04 + ROS 2 already installed. For offline
# handoff bundles, pre-cache NoMachine DEBs with cache_nomachine_debs.sh.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/install_nomachine_remote_access.sh" \
  --yes \
  --install-xfce \
  --open-ufw \
  --auto-headless
