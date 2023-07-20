#!/usr/bin/env bash

if tmux has-session -t ros &>/dev/null; then
  echo "capturing screen output"
  tmux capture-pane -t ros:mid.0 -pS - > ~/tmux-ros-mid.txt
  echo "shutting down tmux session 'ros'"
  tmux send-keys -t ros:mid.0 C-c Enter
else
  echo "no tmux session 'ros' found"
  exit
fi

echo "waiting for shutdown to complete..."
while tmux has-session -t ros &>/dev/null; do
  sleep 1.0
done
echo "shutdown complete"
