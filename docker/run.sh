#!/usr/bin/env bash
# Brings the container up with everything it needs from this host.
#
# Compose cannot discover the host's user id or its GPU group ids, and it
# cannot grant X access, so those happen here and are passed in.
set -e
cd "$(dirname "$0")/.."

if ! command -v docker >/dev/null; then
    echo "docker is not installed - see DOCKER.md" >&2
    exit 1
fi

# Let containers on this machine talk to the X server. Narrower than
# `xhost +`, which would open the display to the whole network.
if command -v xhost >/dev/null; then
    xhost +local:docker >/dev/null
else
    echo "warning: xhost not found; GUI windows may not open" >&2
fi

export USER_UID="$(id -u)"
export USER_GID="$(id -g)"

# Fall back to the common Debian/Ubuntu ids if a group is missing.
export VIDEO_GID="$(getent group video  | cut -d: -f3)"
export RENDER_GID="$(getent group render | cut -d: -f3)"
[ -n "$VIDEO_GID"  ] || export VIDEO_GID=44
[ -n "$RENDER_GID" ] || export RENDER_GID=110

docker compose up -d --build

cat <<'MSG'

Container is running.

  ./docker/shell.sh      open a terminal in it (run this once per terminal)
  ./docker/stop.sh       stop it

First time inside, build the workspace:

  cb

MSG
