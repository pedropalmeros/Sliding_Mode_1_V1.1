#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./Att_Quat_01_rt
else
	EXEC=./Att_Quat_01_nrt
fi

. $FLAIR_ROOT/flair-src/scripts/distribution_specific_hack.sh

$EXEC -n x4_0 -a 127.0.0.1 -p 9000 -l /tmp -x setup_x4.xml -t x4_simu 
