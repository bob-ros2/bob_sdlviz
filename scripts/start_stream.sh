#!/bin/bash
# Copyright 2026 Bob Ros
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# start_stream.sh
# Optimized for workstation use and Twitch streaming using SDLVIZ_* env vars.

# --- Path Detection ---
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -d "$(dirname "$SCRIPT_DIR")/config" ]; then
    PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
elif [ -d "/ros2_ws/src/bob_sdlviz/config" ]; then
    PROJECT_ROOT="/ros2_ws/src/bob_sdlviz"
else
    # Fallback/Default
    PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
fi

# --- Configuration (Standardized SDLVIZ_*) ---
export SDLVIZ_STREAM_PATH="${SDLVIZ_STREAM_PATH:-/tmp/video_pipe}"
export SDLVIZ_AUDIO_PATH="${SDLVIZ_AUDIO_PATH:-/tmp/audio_pipe}"
export SDLVIZ_AUDIO_MASTER_PATH="${SDLVIZ_AUDIO_MASTER_PATH:-/tmp/audio_master_pipe}"
export SDLVIZ_WIDTH="${SDLVIZ_WIDTH:-854}"
export SDLVIZ_HEIGHT="${SDLVIZ_HEIGHT:-480}"
export SDLVIZ_FPS="${SDLVIZ_FPS:-30}"
export SDLVIZ_SHOW_WINDOW="${SDLVIZ_SHOW_WINDOW:-true}"
export SDLVIZ_STREAM_OUTPUT="${SDLVIZ_STREAM_OUTPUT:-true}"
export SDLVIZ_CONFIG_PATH="${SDLVIZ_CONFIG_PATH:-$PROJECT_ROOT/config/sdlviz.json}"
export SDLVIZ_PARAMS_PATH="${SDLVIZ_PARAMS_PATH:-$PROJECT_ROOT/config/sdlviz.yaml}"

export STREAM_KEY="${TWITCH_STREAM_KEY}"
export INGEST_SERVER="${INGEST_SERVER:-rtmp://live-fra.twitch.tv/app/}"

# Path to the ROS setup file.
if [ -f "/ros2_ws/install/setup.bash" ]; then
    DEFAULT_ROS_SETUP="/ros2_ws/install/setup.bash"
else
    DEFAULT_ROS_SETUP="../../install/setup.bash"
fi
ROS_SETUP_PATH="${ROS_SETUP_PATH:-$DEFAULT_ROS_SETUP}"

# --- Sanity Checks ---
if [[ -z "$STREAM_KEY" ]]; then
    echo "Error: TWITCH_STREAM_KEY environment variable is not set."
    exit 1
fi

if [ ! -f "$ROS_SETUP_PATH" ]; then
    echo "Error: ROS 2 setup file not found at '$ROS_SETUP_PATH'."
    exit 1
fi

# --- Sourcing and Setup ---
source "$ROS_SETUP_PATH"
echo "[$(date)] ROS 2 workspace sourced from $ROS_SETUP_PATH."

# Ensure pipes exist
if [[ ! -p $SDLVIZ_STREAM_PATH ]]; then
    mkfifo $SDLVIZ_STREAM_PATH
fi
if [[ ! -p $SDLVIZ_AUDIO_PATH ]]; then
    mkfifo $SDLVIZ_AUDIO_PATH
fi
if [[ ! -p $SDLVIZ_AUDIO_MASTER_PATH ]]; then
    mkfifo $SDLVIZ_AUDIO_MASTER_PATH
fi

cleanup() {
    echo "Shutting down..."
    kill $(jobs -p || echo "")
}
trap cleanup EXIT

# --- Launch Application ---
echo "Starting sdlviz node..."
ros2 run bob_sdlviz sdlviz "$@" &

NODE_PID=$!

# --- Audio Bridge (Mandatory for non-blocking FIFO audio) ---
ENABLE_AUDIO="${ENABLE_AUDIO:-false}"
if [[ "$ENABLE_AUDIO" == "fifo" ]]; then
    echo "Starting audio bridge (Non-blocking)..."
    python3 "$SCRIPT_DIR/audio_bridge.py" &
    BRIDGE_PID=$!
fi

sleep 2

while true
do
    if ! kill -0 $NODE_PID 2>/dev/null; then
        echo "Error: sdlviz node (PID: $NODE_PID) is not running. Restarting node..."
        ros2 run bob_sdlviz sdlviz "$@" &
        NODE_PID=$!
        sleep 2
    fi

    echo "Starting ffmpeg stream to Twitch (Audio: ${ENABLE_AUDIO})..."
    
    AUDIO_ARGS=()
    if [[ "$ENABLE_AUDIO" == "pulse" || "$ENABLE_AUDIO" == "true" ]]; then
        AUDIO_ARGS=(-f pulse -i default -c:a aac -b:a 128k -ar 44100)
    elif [[ "$ENABLE_AUDIO" == "fifo" ]]; then
        # Use the master pipe (fed by audio_bridge.py)
        AUDIO_ARGS=(-f s16le -ar 44100 -ac 2 -i "$SDLVIZ_AUDIO_MASTER_PATH" -c:a aac -b:a 128k)
    else
        # Default or "false"
        AUDIO_ARGS=(-an)
    fi

    # No IPv4 force (-4) as requested
    ffmpeg \
        -f rawvideo -pixel_format bgra -video_size ${SDLVIZ_WIDTH}x${SDLVIZ_HEIGHT} -framerate ${SDLVIZ_FPS} -thread_queue_size 1024 -i $SDLVIZ_STREAM_PATH \
        "${AUDIO_ARGS[@]}" \
        -c:v libx264 -pix_fmt yuv420p -preset ultrafast -tune zerolatency -b:v 3000k -maxrate 3000k -bufsize 6000k \
        -g 60 -r ${SDLVIZ_FPS} \
        -f flv "${INGEST_SERVER}${STREAM_KEY}" || true

    echo "ffmpeg exited. Restarting in 5 seconds..."
    sleep 5
done