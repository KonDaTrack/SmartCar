#!/usr/bin/env bash
# Create the non-root SmartCar user inside the image.
# UID/GID 1000 matches the dev host user and the vehicle 'yunle' user.
set -euo pipefail

USER_NAME="${USER_NAME:-smartcar}"
USER_UID="${USER_UID:-1000}"
USER_GID="${USER_GID:-1000}"

if ! getent group "${USER_NAME}" >/dev/null; then
  if ! getent group "${USER_GID}" >/dev/null; then
    groupadd --gid "${USER_GID}" "${USER_NAME}"
  fi
fi

if ! getent passwd "${USER_NAME}" >/dev/null; then
  useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USER_NAME}"
fi

# Add the user to device groups. Standard Ubuntu GIDs:
# dialout 20, video 44, plugdev 46, render 109.
add_group() {
  local name="$1"
  local gid="$2"
  if ! getent group "${name}" >/dev/null; then
    groupadd --gid "${gid}" "${name}" 2>/dev/null || true
  fi
  if getent group "${name}" >/dev/null; then
    usermod -aG "${name}" "${USER_NAME}" 2>/dev/null || true
  fi
}

add_group dialout 20
add_group video 44
add_group plugdev 46
add_group render 109

mkdir -p /ws
chown "${USER_NAME}:${USER_NAME}" /ws
