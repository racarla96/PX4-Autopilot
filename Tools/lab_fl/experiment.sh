#!/usr/bin/env bash
# One automated flight of the comparison: boot Gazebo SITL, set the gains, take off, run a trajectory under one controller,
# land and store the log.
#
#   experiment.sh <circle|paper|paper2> <lab|fl> <robust|paper1|paper2> [extra "param set" lines, separated by ;]
#
# The log is copied to $PX4_SITL_RT/runs/<trajectory>_<controller>_<gains>.ulg (default ~/px4_sitl_labfl/runs).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
RT=${PX4_SITL_RT:-$HOME/px4_sitl_labfl}
traj=${1:?usage: experiment.sh circle|paper|paper2 lab|fl robust|paper1|paper2 [extra params]}
ctrl=${2:?}
gains=${3:?}
extra=${4:-}
case "$traj" in paper) dur=80 ;; paper2) dur=35 ;; *) dur=60 ;; esac

mkdir -p "$RT/runs"
newest() { find "$ROOT/build/px4_sitl_default_gz/rootfs/log" "$RT/log" -name '*.ulg' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2; }

"$HERE/sitl.sh" start gz "$traj"
"$HERE/sitl.sh" gains "$ctrl" "$gains"
if [ -n "$extra" ]; then
	IFS=';' read -ra items <<< "$extra"
	for p in "${items[@]}"; do "$HERE/sitl.sh" cmd "param set $p"; done
fi
sleep 2
"$HERE/sitl.sh" demo "$ctrl"
sleep $((dur + 8))
"$HERE/sitl.sh" hold
sleep 3
"$HERE/sitl.sh" land
sleep 15
"$HERE/sitl.sh" stop
sleep 3
src=$(newest)
cp "$src" "$RT/runs/${traj}_${ctrl}_${gains}.ulg"
echo "log: $RT/runs/${traj}_${ctrl}_${gains}.ulg"
