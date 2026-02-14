#!/usr/bin/env bash
set -euo pipefail

# Convenience wrapper. Run this from the root of the upstream rpi-image-gen repo.
# Usage:
#   cp ./recipes/ubuntu22-ros2-camera/.env.example ./.env
#   set -a; source ./.env; set +a
#   ./recipes/ubuntu22-ros2-camera/build.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECIPE_DIR="${SCRIPT_DIR}/surgeyos"

# Avoid interactive debconf prompts during chroot package installation.
export DEBIAN_FRONTEND=noninteractive
export DEBCONF_NONINTERACTIVE_SEEN=true
export TZ=Etc/UTC

: "${IGconf_device_layer:=rpi5-ubuntu}"
: "${IGconf_device_user1:=robot}"
: "${IGconf_device_user1pass:=Robot1234!}"
: "${IGconf_wifi_ssid:=}"
: "${IGconf_wifi_psk:=}"
: "${IGconf_ros_variant:=ros-base}"
: "${IGconf_image_name:=ubuntu22-ros2-camera}"
: "${IGconf_image_assetdir:=${RECIPE_DIR}/image/mbr/simple_dual-nosparse}"
: "${IGconf_sys_apt_keydir:=/usr/share/keyrings}"

./rpi-image-gen/rpi-image-gen build -S "$RECIPE_DIR" -c ubuntu22-ros2-camera.yaml -- \
  IGconf_device_layer="$IGconf_device_layer" \
  IGconf_device_user1="$IGconf_device_user1" \
  IGconf_device_user1pass="$IGconf_device_user1pass" \
  IGconf_wifi_ssid="$IGconf_wifi_ssid" \
  IGconf_wifi_psk="$IGconf_wifi_psk" \
  IGconf_ros_variant="$IGconf_ros_variant" \
  IGconf_image_name="$IGconf_image_name" \
  IGconf_image_assetdir="$IGconf_image_assetdir" \
  IGconf_sys_apt_keydir="$IGconf_sys_apt_keydir"
