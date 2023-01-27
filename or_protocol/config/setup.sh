#!/usr/bin/env bash

# setup_odroid
#
# a script for setting up a new mid-adhoc device from a fresh ubuntu image

# helper functions

__error_msg_exit() {
  echo "$1. exiting."
  exit
}

__user_query() {
  while true; do
    echo -n $1" (Y/n) "
    read res

    if [[ "$res" == "y" ]] || [[ "$res" == "Y" ]] || [ -z "$res" ]; then
      return 0
    elif [[ "$res" == "n" ]]; then
      return 1
    fi
  done
}

# parse inputs

if [[ $# -ne 2 ]]; then
  echo "usage: setup.sh <interface> <node-id>"
  exit 1
fi
iface=$1
ip_addr=192.168.0.$2

ifaces=$(ip a | sed -nE 's/^[0-9]+: ([a-z0-9]+):.*/\1/p' | grep -vE "^wl.*" | tr '\n' ' ')
if ! echo $ifaces | grep -qw $iface; then
  echo "'$iface' not in list of wireless interfaces: $ifaces"
  exit
fi

# update timezone

current_timezone=$(sudo timedatectl show | sed -nE 's/^Timezone=(.*)$/\1/p')
desired_timezone=America/New_York
if [[ "$current_timezone" != "$desired_timezone" ]]; then
  echo "updating timezone to $timez"
  sudo timedatectl set-timezone $timez || __error_msg_ext "failed to update timezone"
else
  echo "timezone already $desired_timezone"
fi

# ignore wireless device in network manager

nmignorefile=/etc/NetworkManager/conf.d/99-unmanaged-devices.conf
if [[ -e $nmignorefile ]]; then
  if ! grep -qw $iface $nmignorefile; then
    __error_msg_exit "$nmignorefile already exists but not configured for device '$iface'"
    exit
  else
    echo "$nmignorefile already configured to ignore device '$iface'"
  fi
else
  echo "creating $nmignorefile"
  echo "[keyfile]
unmanaged-devices=interface-name:$iface" > /tmp/unmanaged.conf
  sudo mv /tmp/unmanaged.conf $nmignorefile || __error_msg_exit "failed to create $nmignorefile"
  echo "restarting NetworkManager"
  sudo systemctl restart NetworkManager.service || __error_msg_exit "failed to restart NetworkManager.service"
fi

wpa_file=/var/run/wpa_supplicant/$iface
if [[ -e $wpa_file ]]; then
  echo "$wpa_file still exists, setting up mid-adhoc will fail unless it is removed"
  exit
fi

# create connection to mid-adhoc network

./install.sh $iface $2 || __error_msg_exit "./install.sh script failed"

# ROS setup

if [[ ! -e /opt/ros/noetic ]]; then
  echo -e "\nNO ROS INSTALLATION DETECTED!!!\n"
fi

if [[ ! -e /opt/ros/noetic ]]; then
  echo "making ~/.ros directory"
  mkdir ~/.ros
else
  rcfile=~/.ros/rosconsole.config
  if [[ -e $rcfile ]]; then
    if grep -qE "^log4j\.logger\.ros\.or_protocol" $rcfile; then
      echo "$rcfile already configured for or_protocol"
    else
      echo "$rcfile exists, appending or_protocol settings"
      echo "log4j.logger.ros.or_protocol=DEBUG" >> $rcfile
    fi
  else
    echo "creating $rcfile"
    mv rosconsole.config ~/.ros
  fi
fi

echo "
export ROSCONSOLE_CONFIG_FILE=~/.ros/rosconsole.config
if [[ -e ~/mid_ws/devel/setup.bash ]]; then
  source ~/mid_ws/devel/setup.bash
fi" >> ~/.bashrc

# echo "sudo apt install curl fish tmux python3-catkin-tools gdb"
