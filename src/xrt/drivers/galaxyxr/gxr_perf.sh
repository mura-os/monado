#!/bin/sh
# Pin the Adreno and the display pipeline to performance for Monado testing.
# The tz governor sees the bursty VR workload as idle and drops to 421MHz,
# min_pwrlevel=0 keeps the GPU at the top power level instead.
# Usage (as root): gxr_perf.sh on|off
KGSL=/sys/class/kgsl/kgsl-3d0

case "$1" in
on)
	echo 0 > $KGSL/min_pwrlevel
	echo 1 > /sys/kernel/debug/dri/0/debug/core_perf/perf_mode 2>/dev/null
	echo 1 > /sys/kernel/debug/dri/1/debug/core_perf/perf_mode 2>/dev/null
	;;
off)
	echo 9 > $KGSL/min_pwrlevel
	echo 0 > /sys/kernel/debug/dri/0/debug/core_perf/perf_mode 2>/dev/null
	echo 0 > /sys/kernel/debug/dri/1/debug/core_perf/perf_mode 2>/dev/null
	;;
*)
	echo "usage: $0 on|off" >&2
	exit 1
	;;
esac
echo "GPU min_pwrlevel $(cat $KGSL/min_pwrlevel), now $(cat $KGSL/devfreq/cur_freq)"
