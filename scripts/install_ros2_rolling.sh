#!/usr/bin/env bash
set -euo pipefail

log() { echo -e "\n==> $*\n"; }
die() { echo "ERROR: $*" >&2; exit 1; }

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"
}

# --- Detect OS ---
[[ -f /etc/os-release ]] || die "Cannot detect OS (missing /etc/os-release)."
# shellcheck disable=SC1091
. /etc/os-release

[[ "${ID:-}" == "ubuntu" ]] || die "This script is intended for Ubuntu. Detected: ${ID:-unknown}"

UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
[[ -n "${UBUNTU_CODENAME}" ]] || die "Could not detect Ubuntu codename."
ARCH="$(dpkg --print-architecture)"

# Rolling debs are currently targeted at Ubuntu 24.04 (noble) and common arches like amd64/arm64.
if [[ "${UBUNTU_CODENAME}" != "noble" ]]; then
  die "ROS 2 Rolling debs are intended for Ubuntu 24.04 (noble). Detected: ${UBUNTU_CODENAME}. Use Noble or install from source."
fi
if [[ "${ARCH}" != "amd64" && "${ARCH}" != "arm64" ]]; then
  echo "WARNING: Architecture '${ARCH}' may not be supported by ROS 2 Rolling deb packages." >&2
fi

log "Installing base prerequisites"
sudo apt-get update -o Acquire::Retries=3
sudo apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg lsb-release software-properties-common

# --- Fix/ensure Ubuntu sources are complete (noble + updates + security + backports; all components) ---
enable_ubuntu_sources() {
  local f="/etc/apt/sources.list.d/ubuntu.sources"
  if [[ -f "${f}" ]]; then
    log "Ensuring ${f} includes noble, noble-updates, noble-security, noble-backports + all components"

    # Ensure Suites include the standard pockets
    sudo sed -i -E \
      's/^(Suites:\s*).*/\1noble noble-updates noble-security noble-backports/' \
      "${f}"

    # Ensure Components include the standard components
    sudo sed -i -E \
      's/^(Components:\s*).*/\1main restricted universe multiverse/' \
      "${f}"

    # Ensure it is enabled (remove "Enabled: no" if present)
    sudo sed -i -E \
      's/^Enabled:\s*no/Enabled: yes/' \
      "${f}" || true
  else
    # Fallback for systems still using sources.list
    log "ubuntu.sources not found; enabling standard repositories via add-apt-repository"
    sudo add-apt-repository -y main || true
    sudo add-apt-repository -y restricted || true
    sudo add-apt-repository -y universe || true
    sudo add-apt-repository -y multiverse || true
  fi
}

enable_ubuntu_sources

log "Refreshing apt indexes and repairing any partial installs"
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*
sudo apt-get update -o Acquire::Retries=3

# If there were interrupted installs, fix them
sudo dpkg --configure -a || true
sudo apt-get -f install -y || true

# Unhold packages if any are held (can cause 'not going to be installed')
if [[ -n "$(apt-mark showhold || true)" ]]; then
  log "Removing holds (apt-mark showhold)"
  apt-mark showhold || true
  sudo apt-mark unhold $(apt-mark showhold) || true
fi

log "Upgrading system to align package versions (fixes =version mismatches)"
sudo apt-get update -o Acquire::Retries=3
sudo apt-get full-upgrade -y

# Install a couple of frequently-missing basics explicitly (your error list included these)
log "Ensuring common dependencies are available"
sudo apt-get install -y bzip2 uuid-dev

# --- Locale setup (ROS docs expect UTF-8) ---
log "Configuring locale (en_US.UTF-8)"
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# --- Install ros2-apt-source (sets up ROS repo + keys) ---
log "Installing ros2-apt-source (configures ROS 2 apt repo + keys)"
require_cmd curl
require_cmd awk
require_cmd grep

ROS_APT_SOURCE_VERSION="$(
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" \
    | awk -F\" '{print $4}'
)"
[[ -n "${ROS_APT_SOURCE_VERSION}" ]] || die "Failed to determine latest ros-apt-source release version."

DEB_PATH="/tmp/ros2-apt-source.deb"
DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"

log "Downloading: ${DEB_URL}"
curl -fL "${DEB_URL}" -o "${DEB_PATH}"
sudo dpkg -i "${DEB_PATH}" || sudo apt-get -f install -y

log "Updating apt after adding ROS repo"
sudo apt-get update -o Acquire::Retries=3

# --- Install ROS 2 Rolling ---
log "Installing ROS 2 Rolling (desktop-full if available; otherwise desktop)"
if apt-cache show ros-rolling-desktop-full >/dev/null 2>&1; then
  sudo apt-get install -y ros-rolling-desktop-full
else
  sudo apt-get install -y ros-rolling-desktop
fi

log "Installing ROS development tools"
sudo apt-get install -y ros-dev-tools python3-rosdep

log "Initializing rosdep (safe if already initialized)"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

# --- Environment setup ---
SETUP_LINE="source /opt/ros/rolling/setup.bash"
BASHRC="${HOME}/.bashrc"

log "Adding ROS 2 Rolling sourcing to ${BASHRC} (idempotent)"
grep -Fxq "${SETUP_LINE}" "${BASHRC}" || {
  {
    echo ""
    echo "# ROS 2 Rolling"
    echo "${SETUP_LINE}"
  } >> "${BASHRC}"
}

log "Done."
echo "Open a new terminal, or run:"
echo "  ${SETUP_LINE}"
echo "Test with:"
echo "  ros2 run demo_nodes_cpp talker"
echo "  ros2 run demo_nodes_py listener"