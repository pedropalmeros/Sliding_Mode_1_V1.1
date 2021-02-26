#! /bin/bash
if [ -f /proc/xenomai/version ];then
	EXEC=./uav_single_rt
else
	EXEC=./uav_single_nrt
fi

. $FLAIR_ROOT/flair-src/scripts/distribution_specific_hack.sh

$EXEC -n x4_0 -t x4 -p 9000 -a 127.0.0.1 -x simulator_x4.xml -o 10 -m $FLAIR_ROOT/flair-src/models -s $FLAIR_ROOT/flair-src/models/indoor_flight_arena.xml
