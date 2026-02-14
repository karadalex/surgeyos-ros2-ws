#!/usr/bin/env bash

set -euo pipefail

BUILD_ID=${RANDOM}
RPI_BUILD_SVC="rpi_imagegen"
RPI_BUILD_USER="imagegen"
RPI_CUSTOMIZATIONS_DIR="surgeyos"
RPI_IMAGE_NAME="${IGconf_image_name:-ubuntu22-ros2-camera}"
RPI_CONTAINER_NAME="${RPI_BUILD_SVC}-${BUILD_ID}"

ensure_cleanup() {
  echo "Cleanup containers..."

  RPI_BUILD_SVC_CONTAINER_ID=$(docker ps -a --filter "name=${RPI_CONTAINER_NAME}" --format "{{.ID}}" | head -n 1 || true)
  if [ -n "${RPI_BUILD_SVC_CONTAINER_ID}" ]; then
    docker kill "${RPI_BUILD_SVC_CONTAINER_ID}" >/dev/null 2>&1 || true
    docker rm "${RPI_BUILD_SVC_CONTAINER_ID}" >/dev/null 2>&1 || true
  fi

  echo "Cleanup complete."
}

# Set the trap to execute the ensure_cleanup function on EXIT
trap ensure_cleanup EXIT

mkdir -p "./${RPI_CUSTOMIZATIONS_DIR}/deploy"

# Build a customer raspberry pi image
# with the wifi setup service included
#
echo "🔨 Building Docker image with rpi-image-gen to create ${RPI_BUILD_SVC}..."
docker compose build ${RPI_BUILD_SVC}

# Ensure mounted work volume is clean/writable for the non-root build user.
docker compose run --rm --user root ${RPI_BUILD_SVC} \
  bash -lc "mkdir -p /home/${RPI_BUILD_USER}/work \
    && rm -rf /home/${RPI_BUILD_USER}/work/build \
              /home/${RPI_BUILD_USER}/work/bootstrap \
              /home/${RPI_BUILD_USER}/work/cache \
              /home/${RPI_BUILD_USER}/work/chroot-* \
              /home/${RPI_BUILD_USER}/work/deploy-* \
              /home/${RPI_BUILD_USER}/work/image-* \
    && chown -R ${RPI_BUILD_USER}:${RPI_BUILD_USER} /home/${RPI_BUILD_USER}/work"

echo "🚀 Running image generation in container..."
docker compose run --name "${RPI_CONTAINER_NAME}" -d ${RPI_BUILD_SVC} \
  && docker compose exec ${RPI_BUILD_SVC} bash -c "/home/${RPI_BUILD_USER}/image-build.sh" \
  && CID=$(docker ps -a --filter "name=${RPI_CONTAINER_NAME}" --format "{{.ID}}" | head -n 1) \
  && DEPLOY_DIR=$(docker compose exec ${RPI_BUILD_SVC} bash -lc "ls -dt /home/${RPI_BUILD_USER}/work/deploy-* | head -n 1") \
  && docker cp "${CID}:${DEPLOY_DIR}/${RPI_IMAGE_NAME}.img" "./${RPI_CUSTOMIZATIONS_DIR}/deploy/${RPI_IMAGE_NAME}-$(date +%m-%d-%Y-%H%M).img" \

echo "🚀 Completed -> ${RPI_CUSTOMIZATIONS_DIR}/deploy/${RPI_IMAGE_NAME}-$(date +%m-%d-%Y-%H%M).img"
