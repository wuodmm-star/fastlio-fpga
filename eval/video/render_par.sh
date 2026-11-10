#!/bin/bash
# render_par.sh <worker k> <dump> <log> <out.mp4> <first> <last> : one render worker on its own virtual display
K=$1; shift
source /opt/ros/humble/setup.bash
export CJK_FONT=${CJK_FONT:-/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc}
export ROS_DOMAIN_ID=$((80 + K)) ROS_LOCALHOST_ONLY=1 DISPLAY=:$((100 + K)) LIBGL_ALWAYS_SOFTWARE=1
Xvfb $DISPLAY -screen 0 1600x1000x24 >/dev/null 2>&1 &
XV=$!
sleep 2
rviz2 -d ./fpga_video.rviz > ./rviz_video_$K.log 2>&1 &
RV=$!
sleep 14
python3 -u ./render.py "$@" 1
kill $RV; sleep 1; kill $XV
