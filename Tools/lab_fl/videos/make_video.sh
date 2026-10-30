#!/usr/bin/env bash
# Records a video of a trajectory flown in Gazebo by the lab drone (ZMR250): a wide fixed view with the reference path as dots,
# next to a chase camera fixed to the drone. Output: <out>.mp4 (default videos/zmr250_<traj>_<ctrl>_<gains>.mp4).
#
#   make_video.sh <circle|paper|paper2> <lab|fl> <gains> [out.mp4]
#   e.g. make_video.sh paper2 fl fast2
#
# Needs: the Gazebo SITL build (see the header of ../sitl.sh), ffmpeg, the system python3 with the Gazebo bindings
# (gz.transport13) and a reference path JSON in data/ (data/ref_<traj>.json; make one from a flight log with
# src/ref_path_from_log.py). The model is generated with the chase camera enabled and regenerated without it at the end.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
LABFL=$(cd "$HERE/.." && pwd)
traj=${1:?usage: make_video.sh circle|paper|paper2 lab|fl gains [out.mp4]}
ctrl=${2:?}
gains=${3:?}
out=${4:-$HERE/zmr250_${traj}_${ctrl}_${gains}.mp4}
case "$traj" in paper) dur=80 ;; paper2) dur=35 ;; *) dur=60 ;; esac
ref="$HERE/data/ref_$traj.json"
[ -f "$ref" ] || { echo "missing $ref (see src/ref_path_from_log.py)" >&2; exit 1; }
frames=$(( (14 + dur + 10) * 20 ))   # takeoff 14 s + trajectory + 10 s
tmp=$(mktemp -d)
trap 'python3 "$LABFL/gz/make_model.py" >/dev/null; "$LABFL/sitl.sh" stop >/dev/null 2>&1 || true; rm -rf "$tmp"' EXIT

python3 "$LABFL/gz/make_model.py" --set camera.enabled=true >/dev/null
export VEHICLE=zmr250
"$LABFL/sitl.sh" start gz "$traj" >/dev/null
"$LABFL/sitl.sh" gains "$ctrl" "$gains" >/dev/null
/usr/bin/python3 "$HERE/src/spawn_gz_scene.py" --ref "$ref"
chase=$(gz topic -l | grep chase_cam | head -1)
[ -n "$chase" ] || { echo "chase camera topic not found" >&2; exit 1; }
/usr/bin/python3 "$HERE/src/record_gz_cameras.py" --frames "$frames" "/viewcam=$tmp/wide.mp4" "$chase=$tmp/chase.mp4" > "$tmp/rec.log" 2>&1 &
rec=$!
sleep 3
"$LABFL/sitl.sh" demo "$ctrl" >/dev/null
wait $rec || { cat "$tmp/rec.log" >&2; exit 1; }
"$LABFL/sitl.sh" hold >/dev/null 2>&1 || true

font=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
ctrl_name=$(echo "$ctrl" | tr a-z A-Z)
# wide view scaled to 720 px; chase view scaled to the same height; titles with the simulated time
ffmpeg -y -loglevel error -i "$tmp/wide.mp4" -i "$tmp/chase.mp4" -filter_complex \
  "[0:v]scale=-2:720[a];[1:v]scale=-2:720[b];[a][b]hstack=inputs=2,drawtext=fontfile=$font:text='$ctrl_name - $traj - lab drone ZMR250 (PX4 SITL, Gazebo)   t=%{eif\:t\:d} s':x=20:y=20:fontsize=26:fontcolor=black[v]" \
  -map "[v]" -c:v libx264 -pix_fmt yuv420p -crf 21 -r 20 "$out"
echo "video: $out"
