#!/usr/bin/env bash
# SITL helper for the LAB and FL flight modes.
#
#   sitl.sh start [sih|gz] [circle|paper|paper2]   start PX4 SITL and apply the vehicle calibration
#                                           sih: SIH quadrotor, headless, in Docker (default)
#                                           gz:  Gazebo x500 with the 3D window, on the host (spawned facing north)
#                                           trajectory: circle (yaw constant, default), paper (first trajectory of the
#                                           study, yaw ramp) or paper2 (second one, waypoints, yaw 0 <-> pi)
#   sitl.sh gains lab|fl robust|paper1|paper2|fast2  controller gains: robust = module defaults, fast2 = FL poles -3..-6, paperN = the study's gains
#                                           for its trajectory N (FL with the paper's filtered derivatives; paper2 at Ts = 0.01 s)
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
	if [ "$backend" = circle ] || [ "$backend" = paper ] || [ "$backend" = paper2 ]; then traj=$backend; backend=sih; fi
	stop_all
	mkdir -p "$RT"
	if [ "$backend" = gz ]; then
		[ -x "$GZBUILD/bin/px4" ] || { echo "build the Gazebo SITL first (see the header of this script)" >&2; exit 1; }
		rm -f "$RT/cmd" "$RT/px4.log" "$RT/parameters.bson" "$RT/parameters_backup.bson" # always start from the default parameters
		mkfifo "$RT/cmd"
		ln -sf "$GZBUILD/rootfs/gz_env.sh" "$RT/gz_env.sh"
		# read-write open of the FIFO: it never sees EOF when a writer closes.
		# No arguments, as `make px4_sitl_default gz_x500` does: PX4 links its own etc directory into the working directory.
		# setsid makes PX4 the leader of its process group, so `stop` can kill PX4 and the Gazebo processes it starts.
		# The x500 of Gazebo spawns facing east (heading 90 deg). The LAB and FL laws assume the vehicle faces north, so spawn it
		# facing north unless the caller sets PX4_GZ_MODEL_POSE ("x,y,z,roll,pitch,yaw" in the ENU world frame, yaw pi/2 = north).
		export PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE:-0,0,0.1,0,0,1.5708}
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
	paper2) send "param set TRJ_TYPE 4" ;;
	*) send "param set TRJ_TYPE 1"; send "param set TRJ_DURATION 60" ;;
	esac
	wait_ready
	echo "PX4 SITL ($backend) is running (trajectory: $traj). Open QGroundControl, then: $0 demo lab"
	;;
gains)
	ctrl=${2:?usage: gains lab|fl robust|paper1|paper2}
	set_ts=0.02
	list=""
	case "$ctrl:${3:-robust}" in
	lab:robust | lab:paper1) list="LAB_K1 0.99|LAB_K2 0.97|LAB_K3 0.99|LAB_K4 0.98|LAB_K5 0.99|LAB_K6 0.79|LAB_K7 0.96|LAB_K8 0.91|LAB_K9 0.96|LAB_K10 0.92|LAB_K11 0.99|LAB_K12 0.95" ;;
	lab:paper2) set_ts=0.01; list="LAB_K1 0.99|LAB_K2 0.89|LAB_K3 0.99|LAB_K4 0.89|LAB_K5 0.99|LAB_K6 0.79|LAB_K7 0.85|LAB_K8 0.81|LAB_K9 0.85|LAB_K10 0.81|LAB_K11 0.85|LAB_K12 0.95" ;;
	fl:robust) list="FL_K11 7.5|FL_K12 19.25|FL_K13 17.75|FL_K14 7.0|FL_K21 7.5|FL_K22 19.25|FL_K23 17.75|FL_K24 7.0|FL_K31 7.5|FL_K32 19.25|FL_K33 17.75|FL_K34 7.0|FL_K41 0.5|FL_K42 1.5|FL_MODEL_DERIV 1" ;;
	fl:fast2) set_ts=0.01; list="FL_K11 360|FL_K12 342|FL_K13 119|FL_K14 18|FL_K21 360|FL_K22 342|FL_K23 119|FL_K24 18|FL_K31 360|FL_K32 342|FL_K33 119|FL_K34 18|FL_K41 18|FL_K42 6|FL_MODEL_DERIV 1" ;;
	fl:paper1) list="FL_K11 1.452|FL_K12 3.509|FL_K13 2.41|FL_K14 2.9|FL_K21 1.26|FL_K22 2.5|FL_K23 2.26|FL_K24 2.5|FL_K31 1.6698|FL_K32 3.509|FL_K33 2.59|FL_K34 2.9|FL_K41 0.5|FL_K42 1.5|FL_MODEL_DERIV 0" ;;
	fl:paper2) set_ts=0.01; list="FL_K11 0|FL_K12 49.375|FL_K13 6.25|FL_K14 7.9|FL_K21 0|FL_K22 50|FL_K23 6.25|FL_K24 8|FL_K31 0|FL_K32 58.32|FL_K33 7.29|FL_K34 8|FL_K41 18|FL_K42 6|FL_MODEL_DERIV 0" ;;
	*) echo "usage: gains lab|fl robust|paper1|paper2" >&2; exit 1 ;;
	esac
	send "param set TCC_TS $set_ts"
	IFS='|' read -ra items <<< "$list"
	for p in "${items[@]}"; do send "param set $p"; done
	echo "gains $ctrl ${3:-robust} applied (TCC_TS $set_ts)"
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
