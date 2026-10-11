#!/usr/bin/env bash
# One terminal inside the running container. Run it again in another host
# terminal for each extra ROS terminal you need.
cd "$(dirname "$0")/.."
exec docker compose exec dev bash
