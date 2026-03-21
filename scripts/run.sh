#!/usr/bin/env bash
set -euo pipefail

python -m src.main --config config/vehicle.yaml "$@"

