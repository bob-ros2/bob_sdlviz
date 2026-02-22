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

# @file write_fifo.sh
# @brief Helper script to read from stdin and write to a named pipe (FIFO).

usage() {
  echo "Usage: $0 --path <fifo_path>"
  echo "Example: ffmpeg ... -f rawvideo - | $0 --path /tmp/my_fifo"
  exit 1
}

FIFO_PATH=""

# Parse arguments
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --path) FIFO_PATH="$2"; shift ;;
    -h|--help) usage ;;
    *) echo "Unknown parameter: $1"; usage ;;
  esac
  shift
done

if [ -z "$FIFO_PATH" ]; then
  usage
fi

# Ensure the directory exists
mkdir -p "$(dirname "$FIFO_PATH")"

# Create FIFO if it doesn't exist
if [ ! -p "$FIFO_PATH" ]; then
  echo "Creating FIFO at $FIFO_PATH"
  mkfifo "$FIFO_PATH"
fi

# Redirect stdin to the pipe
# This will block until a reader (sdlviz) opens the pipe.
cat > "$FIFO_PATH"
