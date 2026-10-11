#!/usr/bin/env bash
# Runs before the container's main process. Interactive shells opened later
# with `docker compose exec` bypass this and are set up by ~/.bashrc instead,
# so the two have to stay in step.
set -e

source /opt/ros/jazzy/setup.bash

# Only present once the workspace has been built at least once.
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

export GZ_SIM_RESOURCE_PATH=/ws/install/field_robot/share/field_robot/models:${GZ_SIM_RESOURCE_PATH}

# Qt warns on every start without this directory, and some plugins refuse to
# load. It has to be private to the user or Qt rejects it.
mkdir -p "${XDG_RUNTIME_DIR:-/tmp/runtime-ros}"
chmod 700 "${XDG_RUNTIME_DIR:-/tmp/runtime-ros}"

exec "$@"
