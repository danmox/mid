#!/usr/bin/env bash

if tmux has-session -t ros &>/dev/null; then
  echo "sending C-c to tmux pane"
  tmux send-keys -t ros:mid.0 C-c Enter
  echo "capturing screen output"
  tmux capture-pane -t ros:mid.0 -pS - > ~/tmux-ros-mid.txt
else
  echo "no tmux session 'ros' found"
  exit
fi

echo "waiting for shutdown to complete..."
while tmux has-session -t ros &>/dev/null; do
  PANE_DEAD=$(tmux list-panes -t ros:mid.0 -F "#{pane_dead}")
  if [[ "$PANE_DEAD" == "1" ]]; then
    echo "tmux pane dead, killing session"
    tmux kill-session -t ros
  fi
  sleep 1.0
done
echo "shutdown complete"
