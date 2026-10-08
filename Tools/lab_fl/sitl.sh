#!/usr/bin/env bash
# SITL helper for the LAB and FL flight modes (SIH quadrotor, no Gazebo, runs in Docker with the host network so that
# QGroundControl on the host connects by itself on UDP 14550).
#
#   sitl.sh start [circle|paper]   start PX4 SITL and apply the SIH calibration (default trajectory: circle)
#   sitl.sh demo lab|fl            arm, take off and switch to the mode
#   sitl.sh hold                   safe exit: back to Hold
#   sitl.sh land                   land
#   sitl.sh cmd "<px4 shell cmd>"  any PX4 shell command, e.g. "traj_generator status"
#   sitl.sh log                    follow the PX4 console
#   sitl.sh stop                   stop the simulation
#
# Logs (.ulg) are written to $PX4_SITL_RT/log (default ~/px4_sitl_labfl). Build first:
#   docker run --rm -u $(id -u):$(id -g) -v "$PWD":/src -w /src -e HOME=/tmp px4io/px4-dev:v1.17.0 make px4_sitl_default
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/../.." && pwd)
RT=${PX4_SITL_RT:-$HOME/px4_sitl_labfl}
NAME=px4_labfl
IMAGE=px4io/px4-dev:v1.17.0

send() { echo "$*" > "$RT/cmd"; }

wait_boot() {
	for _ in $(seq 1 90); do
		grep -q "Ready for takeoff" "$RT/px4.log" 2>/dev/null && return 0
		sleep 1
	done
	echo "PX4 did not report 'Ready for takeoff' in 90 s, see $RT/px4.log" >&2
	return 1
}

case "${1:-}" in
start)
	[ -x "$ROOT/build/px4_sitl_default/bin/px4" ] || { echo "build px4_sitl_default first (see the header of this script)" >&2; exit 1; }
	docker rm -f "$NAME" >/dev/null 2>&1 || true
	mkdir -p "$RT"
	docker run -d --rm --name "$NAME" --network host -u "$(id -u):$(id -g)" \
		-v "$ROOT":/src -v "$RT":/tmp/rt -v "$ROOT/Tools/lab_fl":/harness -w /src/build/px4_sitl_default \
		-e HOME=/tmp -e PX4_SYS_AUTOSTART=10040 -e PX4_SIM_MODEL=sihsim_quadx -e JOYSTICK="${JOYSTICK:-1}" \
		"$IMAGE" /harness/container_run.sh >/dev/null
	echo "waiting for PX4 to boot..."
	wait_boot
	# model of the SIH quadrotor (see the TCC_* parameter documentation)
	for p in "TCC_MASS 1.0" "TCC_IXX 0.025" "TCC_IYY 0.025" "TCC_IZZ 0.03" "TCC_TQ_MAX_XY 2.83" "TCC_TQ_MAX_Z 0.4"; do
		send "param set $p"
	done
	case "${2:-circle}" in
	paper) send "param set TRJ_TYPE 3" ;;
	*) send "param set TRJ_TYPE 1"; send "param set TRJ_DURATION 60" ;;
	esac
	echo "PX4 SITL is running (trajectory: ${2:-circle}). Open QGroundControl, then: $0 demo lab"
	;;
demo)
	mode=${2:?usage: demo lab|fl}
	send "commander arm -f"
	send "commander takeoff"
	echo "taking off, switching to $mode in 14 s..."
	sleep 14
	send "commander mode $mode"
	;;
hold) send "commander mode auto:loiter" ;;
land) send "commander land" ;;
cmd) shift; send "$*" ;;
log) tail -n 40 -f "$RT/px4.log" ;;
stop) docker rm -f "$NAME" >/dev/null 2>&1 && echo stopped || echo "not running" ;;
*) sed -n '2,15p' "$0"; exit 1 ;;
esac
