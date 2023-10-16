#!/bin/bash
# This script is used to run the UDP ros node

SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)  #作用是什么？ 获取当前脚本所在目录
cd $(dirname $SHELL_FOLDER) #作用是什么？ 进入当前脚本所在目录的上一级目录
. install/setup.bash
export LD_LIBRARY_PATH=$(dirname $SHELL_FOLDER)/src/unitree_legged_real/src/unitree_legged_sdk/lib:$LD_LIBRARY_PATH

ros2 run unitree_legged_real ros2_udp lowlevel
