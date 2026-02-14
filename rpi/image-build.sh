#!/usr/bin/env bash
set -euo pipefail

# Convenience wrapper. Run this from the root of the upstream rpi-image-gen repo.
# Usage:
#   cp ./recipes/ubuntu22-ros2-camera/.env.example ./.env
#   set -a; source ./.env; set +a
#   ./recipes/ubuntu22-ros2-camera/build.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECIPE_DIR="${SCRIPT_DIR}/surgeyos"

: "${IGconf_device_layer:=rpi5}"
: "${IGconf_device_user1:=robot}"
: "${IGconf_device_user1pass:=changeme}"
: "${IGconf_wifi_ssid:=}"
: "${IGconf_wifi_psk:=}"
: "${IGconf_ros_variant:=ros-base}"
: "${IGconf_image_name:=ubuntu22-ros2-camera}"

./rpi-image-gen/rpi-image-gen build -S "$RECIPE_DIR" -c ubuntu22-ros2-camera.yaml -- \
  IGconf_device_layer="$IGconf_device_layer" \
  IGconf_device_user1="$IGconf_device_user1" \
  IGconf_device_user1pass="$IGconf_device_user1pass" \
  IGconf_wifi_ssid="$IGconf_wifi_ssid" \
  IGconf_wifi_psk="$IGconf_wifi_psk" \
  IGconf_ros_variant="$IGconf_ros_variant" \
  IGconf_image_name="$IGconf_image_name"
