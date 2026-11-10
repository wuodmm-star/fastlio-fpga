#!/bin/bash
# usage: run_fastlio.sh <mode cpu|fixed|fpga> <tag> [rate] [dump(0/1)]
# plays NTU-VIRAL eee_03 through FAST-LIO with the fpga_hth hook; writes ./<tag>.tum (+ <tag>.dump)
MODE=$1; TAG=$2; RATE=${3:-0.5}; DUMP=${4:-0}
OUT=$(pwd)
source /opt/ros/humble/setup.bash; source ~/fastlio_ws/install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
export FPGA_HTH_MODE=$MODE FPGA_HTH_TRAJ=$OUT/$TAG.tum
[ "$DUMP" = 1 ] && export FPGA_HTH_DUMP=$OUT/$TAG.dump
rm -f $OUT/$TAG.tum $OUT/$TAG.dump
ros2 run fast_lio fastlio_mapping --ros-args --params-file ~/fastlio_ws/src/FAST_LIO/config/ntu_viral.yaml $EXTRA_ARGS > $OUT/$TAG.node.log 2>&1 &
NODE=$!
sleep 3
ros2 bag play ~/data/eee_03_ros2 -r $RATE --topics /os1_cloud_node1/points /imu/imu > $OUT/$TAG.play.log 2>&1
sleep 5
kill -INT $NODE; sleep 3; kill -9 $NODE 2>/dev/null
echo "$TAG: $(wc -l < $OUT/$TAG.tum) poses"; grep fpga_hth $OUT/$TAG.node.log | tail -2
