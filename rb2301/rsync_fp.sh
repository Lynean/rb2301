#!/usr/bin/env bash
set -euo pipefail

robot="${1:-}"

if [[ ! "$robot" =~ ^[0-9]{1,2}$ ]]; then
    echo "Usage: $0 <robot number, 0–99>"
    echo "Example: $0 3"
    exit 1
fi

printf -v nn '%02d' "$((10#$robot))"
target="bingda@192.168.1.2${nn}"

rsync -auvx --delete \
    /home/nhien/rb2301/rb2301/src/rb2301_fp \
    "${target}:~/Downloads/rb2301_fp_13/src"

rsync -auvx \
    /home/nhien/rb2301/rb2301/src/rb2301_fp/fp.sh \
    /home/nhien/rb2301/rb2301/src/rb2301_fp/obstacle_tracker.sh \
    /home/nhien/rb2301/rb2301/src/rb2301_fp/blindzone_check.sh \
    "${target}:~/Downloads/rb2301_fp_13"
