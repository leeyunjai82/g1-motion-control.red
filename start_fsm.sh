#!/bin/bash
if [ "$#" -ne 1 ] || { [ "$1" != "stand" ] && [ "$1" != "sit" ]; }; then
    echo "Usage: $0 [stand|sit]"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo /home/circulus/miniconda3/envs/tv/bin/python "$SCRIPT_DIR/utils/init_fsm.py" "$1"
