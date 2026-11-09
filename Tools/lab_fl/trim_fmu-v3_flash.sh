#!/usr/bin/env bash
# Frees about 126 KB of flash on the fmu-v3 (97.24 % -> 91.25 % used, measured on the base v1.17.0 build) by removing the
# modules a multicopter does not use: fixed wing, VTOL, underwater vehicles, airspeed selector and the fake GPS example.
# Use it only when a build no longer fits in the 2 MB of flash (see the README).
#
#   trim_fmu-v3_flash.sh [board file]     default: boards/px4/fmu-v3/default.px4board
#
# Build afterwards:  make px4_fmu-v3_default   and check the size with arm-none-eabi-size (text + data).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
file=${1:-$ROOT/boards/px4/fmu-v3/default.px4board}
for m in MODULES_AIRSPEED_SELECTOR MODULES_FW_ATT_CONTROL MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL MODULES_FW_MODE_MANAGER \
	MODULES_FW_LATERAL_LONGITUDINAL_CONTROL MODULES_FW_RATE_CONTROL MODULES_UUV_ATT_CONTROL MODULES_UUV_POS_CONTROL \
	MODULES_VTOL_ATT_CONTROL EXAMPLES_FAKE_GPS; do
	sed -i "/^CONFIG_${m}=y\$/d" "$file"
done
echo "removed the fixed-wing, VTOL, UUV, airspeed and fake GPS modules from $file"
