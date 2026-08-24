#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"
exec python3 web_server.py --host "${RMF_LAB_HOST:-0.0.0.0}" --port "${RMF_LAB_PORT:-8080}"
