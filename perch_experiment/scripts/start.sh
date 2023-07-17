#!/usr/bin/env bash

TYPE=task
if [[ "$#" -eq 1 ]]; then
  TYPE="$1"
fi

if tmux has-session -t ros &>/dev/null; then
  echo "tmux session 'ros' already running!"
  exit
fi

if [[ -z "$AGENT" ]]; then
  echo "AGENT environment variable not set"
  exit
fi

MID_ID=${AGENT:(-2)}
ROS_IP=192.168.0.$MID_ID
ROS_MASTER_URI=http://$ROS_IP:11311

echo "starting robot $MID_ID with type: $TYPE"
tmux new-session -d -s ros -n mid "source ~/mid_ws/devel/setup.bash; export ROS_IP=$ROS_IP; export ROS_MASTER_URI=http://$ROS_IP:11311; roslaunch perch_experiment scarab.launch type:=$TYPE"

echo "sleeping for 5.0 seconds to let nodes start up"
sleep 5.0

echo "checking if xtion started successfully..."
source ~/mid_ws/devel/setup.bash
export ROS_IP=$ROS_IP
export ROS_MASTER_URI=$ROS_MASTER_URI

for i in {1..2}; do
  echo "attempt $i"
  timeout 5 rostopic echo -n1 --noarr /$AGENT/scan_merged/header/seq

  RES=$?
  if [[ "$RES" -eq 124 ]]; then
    echo "timeout reached waiting for xtion"
  elif [[ "$RES" -eq 0 ]]; then
    echo "everything looks good!"
    exit
  else
    echo "RES=$RES; robot may not have started successfully"
    exit
  fi

  echo "sleeping for 5.0 seconds before trying again..."
  sleep 5.0
done
