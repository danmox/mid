#!/usr/bin/env bash

if tmux has-session -t ros &>/dev/null; then
  echo "shutting down tmux session 'ros'"
  tmux send-keys -t ros:mid.0 C-c Enter
else
  echo "no tmux session 'ros' found"
fi
