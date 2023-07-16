#!/usr/bin/env bash

TYPE=task
if [[ "$#" -eq 1 ]]; then
  TYPE="$1"
fi

if tmux has-session -t ros &>/dev/null; then
  echo "tmux sesesion 'ros' already running!"
  exit
fi

MID_ID=${AGENT:(-2)}
ROS_IP=192.168.0.$MID_ID

echo "starting robot $MID_ID with type: $TYPE"
tmux new-session -d -s ros -n mid "source ~/mid_ws/devel/setup.bash; export ROS_IP=$ROS_IP; export ROS_MASTER_URI=http://$ROS_IP:11311; roslaunch perch_experiment scarab.launch type:=$TYPE"

echo "waiting for 10s to check if startup was successful"
sleep 10.0

if tmux has-session -t ros &>/dev/null; then
  echo "successfully started robot"
else
  echo "robot not running!"
fi
