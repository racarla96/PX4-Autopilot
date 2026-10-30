#!/usr/bin/env bash
# Flies the second trajectory of the paper with LAB and FL for several time constants of the rotor speed lag, to see how much
# the controllers depend on it (the lag of the real motor + propeller is not identified: the bench sweep is quasi-static).
#
#   sweep_motor_lag.sh [tau ...]        default: 0.0125 0.02 0.03 0.05  (seconds)
#
# Logs: ~/px4_sitl_labfl/runs/zmr250/paper2_<lab|fl>_<gains>_tau<ms>.ulg. The model is left generated with the last tau;
# run make_model.py again to restore the value of zmr250.toml.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
RT=${PX4_SITL_RT:-$HOME/px4_sitl_labfl}
taus=("$@")
[ ${#taus[@]} -gt 0 ] || taus=(0.0125 0.02 0.03 0.05)
for tau in "${taus[@]}"; do
	python3 "$HERE/make_model.py" --set "motor.time_constant=$tau" >/dev/null
	ms=$(python3 -c "print(round($tau*1000))")
	for run in "lab paper2" "fl fast2"; do
		set -- $run
		VEHICLE=zmr250 "$HERE/../experiment.sh" paper2 "$1" "$2" >/dev/null
		mv "$RT/runs/zmr250/paper2_$1_$2.ulg" "$RT/runs/zmr250/paper2_$1_$2_tau${ms}.ulg"
		echo "done: tau ${ms} ms, $1 $2"
	done
done
python3 "$HERE/make_model.py" >/dev/null
