#!/usr/bin/env bash
# Stops the container. Build output lives in named volumes and survives, so
# the next start does not recompile. Add -v to throw that away too.
cd "$(dirname "$0")/.."
docker compose down "$@"
