#!/usr/bin/env bash
# SITL helper for the LAB and FL flight modes.
#
#   sitl.sh start [sih|gz] [circle|paper]   start PX4 SITL and apply the vehicle calibration
#                                           sih: SIH quadrotor, headless, in Docker (default)
#                                           gz:  Gazebo x500 with the 3D window, on the host
#                                           trajectory: circle (yaw constant, default) or paper
#   sitl.sh demo lab|fl                     arm, take off and switch to the mode
#   sitl.sh hold                            safe exit: back to Hold
#   sitl.sh land                            land
#   sitl.sh cmd "<px4 shell cmd>"           any PX4 shell command, e.g. "traj_generator status"
#   sitl.sh log                             follow the PX4 console
#   sitl.sh stop                            stop the simulation
#
# Both use the host network, so QGroundControl on the host connects by itself (UDP 14550).
# Logs (.ulg) are written to $PX4_SITL_RT/log (default ~/px4_sitl_labfl).
#
# Builds:
#   sih: docker run --rm -u $(id -u):$(id -g) -v "$PWD":/src -w /src -e HOME=/tmp px4io/px4-dev:v1.17.0 make px4_sitl_default
#   gz:  make px4_sitl_default BUILD_DIR_SUFFIX=_gz   (needs Gazebo Harmonic, see Tools/lab_fl/README.md)
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/../.." && pwd)
RT=${PX4_SITL_RT:-$HOME/px4_sitl_labfl}
NAME=px4_labfl
JOY=px4_labfl_joy
IMAGE=px4io/px4-dev:v1.17.0
GZBUILD=$ROOT/build/px4_sitl_default_gz

send() { echo "$*" > "$RT/cmd"; }

wait_startup() {
	for _ in $(seq 1 240); do
		grep -q "Startup script returned successfully" "$RT/px4.log" 2>/dev/null && return 0
		sleep 1
	done
	echo "PX4 did not finish its startup script in 240 s, see $RT/px4.log" >&2
	return 1
}

wait_ready() {
	for _ in $(seq 1 90); do
		grep -q "Ready for takeoff" "$RT/px4.log" 2>/dev/null && return 0
		sleep 1
	done
	echo "warning: PX4 did not report 'Ready for takeoff' in 90 s (see $RT/px4.log), continuing" >&2
}

stop_all() {
	docker rm -f "$NAME" "$JOY" >/dev/null 2>&1 || true
	if [ -f "$RT/px4.pid" ]; then
		kill -- "-$(cat "$RT/px4.pid")" >/dev/null 2>&1 || true
		rm -f "$RT/px4.pid"
	fi
}

calibration() {
	# model of the vehicle (see the TCC_* parameter documentation)
	case "$1" in
	gz) set -- "TCC_MASS 2.06" "TCC_IXX 0.0236" "TCC_IYY 0.0236" "TCC_IZZ 0.0439" "TCC_TQ_MAX_XY 3.14" "TCC_TQ_MAX_Z 0.52" ;;
	*) set -- "TCC_MASS 1.0" "TCC_IXX 0.025" "TCC_IYY 0.025" "TCC_IZZ 0.03" "TCC_TQ_MAX_XY 2.83" "TCC_TQ_MAX_Z 0.4" ;;
	esac
	# no data link loss action: the preflight checks must not depend on a ground station being connected
	for p in "$@" "NAV_DLL_ACT 0"; do send "param set $p"; done
}

case "${1:-}" in
start)
	backend=${2:-sih}
	traj=${3:-circle}
	if [ "$backend" = circle ] || [ "$backend" = paper ]; then traj=$backend; backend=sih; fi
	stop_all
	mkdir -p "$RT"
	if [ "$backend" = gz ]; then
		[ -x "$GZBUILD/bin/px4" ] || { echo "build the Gazebo SITL first (see the header of this script)" >&2; exit 1; }
		rm -f "$RT/cmd" "$RT/px4.log"
		mkfifo "$RT/cmd"
		ln -sf "$GZBUILD/rootfs/gz_env.sh" "$RT/gz_env.sh"
		# read-write open of the FIFO: it never sees EOF when a writer closes.
		# No arguments, as `make px4_sitl_default gz_x500` does: PX4 links its own etc directory into the working directory.
		# setsid makes PX4 the leader of its process group, so `stop` can kill PX4 and the Gazebo processes it starts.
		(cd "$RT"
			GZ_IP=127.0.0.1 PX4_SIM_MODEL=gz_x500 setsid nohup "$GZBUILD/bin/px4" <> "$RT/cmd" > "$RT/px4.log" 2>&1 &
			echo $! > "$RT/px4.pid")
		if [ "${JOYSTICK:-1}" = "1" ]; then
			docker run -d --rm --name "$JOY" --network host -v "$ROOT/Tools/lab_fl":/harness "$IMAGE" python3 /harness/mc_send.py >/dev/null
		fi
	else
		[ -x "$ROOT/build/px4_sitl_default/bin/px4" ] || { echo "build px4_sitl_default first (see the header of this script)" >&2; exit 1; }
		docker run -d --rm --name "$NAME" --network host -u "$(id -u):$(id -g)" \
			-v "$ROOT":/src -v "$RT":/tmp/rt -v "$ROOT/Tools/lab_fl":/harness -w /src/build/px4_sitl_default \
			-e HOME=/tmp -e PX4_SYS_AUTOSTART=10040 -e PX4_SIM_MODEL=sihsim_quadx -e JOYSTICK="${JOYSTICK:-1}" \
			"$IMAGE" /harness/container_run.sh >/dev/null
	fi
	echo "waiting for PX4 to boot ($backend), this takes up to a couple of minutes..."
	wait_startup
	calibration "$backend"
	case "$traj" in
	paper) send "param set TRJ_TYPE 3" ;;
	*) send "param set TRJ_TYPE 1"; send "param set TRJ_DURATION 60" ;;
	esac
	wait_ready
	echo "PX4 SITL ($backend) is running (trajectory: $traj). Open QGroundControl, then: $0 demo lab"
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
stop) stop_all; echo stopped ;;
*) sed -n '2,20p' "$0"; exit 1 ;;
esac
