#!/bin/bash
# Runs inside the px4io/px4-dev container: PX4 SITL (SIH quadrotor) whose shell reads commands from a FIFO.
cd /tmp/rt || exit 1
rm -f cmd px4.log
mkfifo cmd
exec 3<>cmd   # read-write open: the FIFO never sees EOF when a writer closes

if [ "${JOYSTICK:-1}" = "1" ]; then
	python3 /harness/mc_send.py &
fi

exec /src/build/px4_sitl_default/bin/px4 /src/build/px4_sitl_default/etc -s etc/init.d-posix/rcS -t /src/test_data <&3 > px4.log 2>&1
