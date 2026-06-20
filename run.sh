#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="$HOME/.local/bin"
POLICY="${1:-dream_waq/dream_waq}"
SESSION="ares"

# Kill existing session if any
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Start RouDi if not running
if ! pgrep -x iox-roudi > /dev/null 2>&1; then
    echo "[run] Starting iox-roudi..."
    iox-roudi &
    sleep 2
fi

# Create tmux session with two panes
tmux new-session -d -s "$SESSION" -x 200 -y 50

# Pane 0: ares_driver_node
tmux send-keys -t "$SESSION" "$BIN_DIR/ares_driver_node $POLICY" C-m

# Pane 1: ares (RL node)
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" "$BIN_DIR/ares $POLICY" C-m

# Select first pane
tmux select-pane -t "$SESSION":0

# Attach to session
tmux attach-session -t "$SESSION"
