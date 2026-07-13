#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
g++ -std=c++17 -Wall -O2 test_mapping.cc joystick.cc -o test_mapping
echo "Built: $SCRIPT_DIR/test_mapping"
echo "Run with:   ./test_mapping              (defaults to /dev/input/js0)"
echo "       or:  ./test_mapping /dev/input/js0"
