#!/usr/bin/env bash

if [ -f /dcist_ws/install/setup.bash ]; then
  echo "Setting up catkin ws: /dcist_ws/install/setup.sh"
  echo "source /dcist_ws/install/setup.sh" >> /$HOME/.bashrc
  source /$HOME/.bashrc
  source /dcist_ws/install/setup.sh
fi

if [ "$#" -eq 0 ]; then
  exec bash
else
  exec "$@"
fi
