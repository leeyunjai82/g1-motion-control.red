#!/bin/bash
if [ "$#" -ne 1 ] || { [ "$1" != "stand" ] && [ "$1" != "sit" ] && [ "$1" != "bal" ] && [ "$1" != "no-bal" ]; }; then
    echo "Usage: $0 [stand|sit|bal|no-bal]"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo /home/circulus/miniconda3/envs/tv/bin/python "$SCRIPT_DIR/utils/init_fsm.py" "$1"
