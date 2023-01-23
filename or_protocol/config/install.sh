#!/usr/bin/env bash

__error_msg_exit() {
	echo "$1. exiting."
	exit
}

# parse inputs

if [[ $# -ne 2 ]]; then
	echo "usage: install <interface> <node-id>"
	exit 1
fi
iface=$1
ip_addr=192.168.0.$2

# double check that everything looks right with user before installing

while true; do
  echo -n "Configure ad-hoc network for interface '$iface' with IP '$ip_addr' (Y/n)? "
  read res

  if [[ "$res" == "y" ]] || [[ "$res" == "Y" ]] || [ -z "$res" ]; then
    break
  elif [[ "$res" == "n" ]]; then
    exit
  fi
done

# correct absolute paths in mid-adhoc@.service

rm_name=$(which rm 2>/dev/null) || __error_msg_exit "'rm' not found"
rfkill_name=$(which rfkill 2>/dev/null) || __error_msg_exit "'rfkill' not found"
ip_name=$(which ip 2>/dev/null) || __error_msg_exit "'ip' not found"
wpa_name=$(which wpa_supplicant 2>/dev/null) || __error_msg_exit "'wpa_supplicant' not found"

if [[ -e mid-adhoc@.service ]]; then
  cp mid-adhoc@.service tmp@.service
else
  echo "mid-adhoc@.service does not exist in the current directory."
  exit
fi


sed -i "s|rm_path|$rm_name|" tmp@.service
sed -i "s|rfkill_path|$rfkill_name|" tmp@.service
sed -i "s|ip_path|$ip_name|" tmp@.service
sed -i "s|wpa_supplicant_path|$wpa_name|" tmp@.service

# generate config file

echo "creating 25-mid-adhoc.network"
echo "[Match]
Name=$iface

[Network]
Address=$ip_addr/24
Gateway=192.168.0.101
DNS=8.8.8.8" > 25-mid-adhoc.network

# copy service files to system folder

echo "installing mid-adhoc-open.conf to /etc/wpa_supplicant/"
sudo cp mid-adhoc-open.conf /etc/wpa_supplicant/
echo "installing mid-adhoc@.service to /etc/systemd/system/"
sudo mv tmp@.service /etc/systemd/system/mid-adhoc@.service
echo "installing 25-mid-adhoc.network to /etc/systemd/network/"
sudo mv 25-mid-adhoc.network /etc/systemd/network/

# enable system services

if sudo systemctl is-enabled --quiet mid-adhoc@$iface.service; then
  echo "reloading system daemons"
  sudo systemctl daemon-reload || __error_msg_exit "failed to reload daemon files"
else
  echo "enabling system services"
  sudo systemctl enable mid-adhoc@$iface.service || __error_msg_exit "failed to enable mid-adhoc@$iface.service"
  sudo systemctl enable systemd-networkd.service || __error_msg_exit "failed to enable systemd-networkd.service"
fi
echo "restarting mid-adhoc@$iface.service and systemd-networkd.service"
sudo systemctl restart mid-adhoc@$iface.service || __error_msg_exit "failed to restart mid-adhoc@$iface.service"
sudo systemctl restart systemd-networkd.service || __error_msg_exit "failed to restart systemd-networkd.service"
