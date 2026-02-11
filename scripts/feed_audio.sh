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
# feed_audio.sh
# Helper script to feed audio from various sources into the SDLVIZ_AUDIO_PATH FIFO.

AUDIO_PATH="${SDLVIZ_AUDIO_PATH:-/tmp/audio_pipe}"

if [[ ! -p "$AUDIO_PATH" ]]; then
    echo "Error: Audio FIFO not found at $AUDIO_PATH. Is start_stream.sh running?"
    exit 1
fi

SOURCE="$1"
if [[ -z "$SOURCE" ]]; then
    echo "Usage: ./feed_audio.sh <audio_source>"
    echo "Examples:"
    echo "  ./feed_audio.sh music.mp3"
    echo "  ./feed_audio.sh http://example.com/radio.mp3"
    echo "  ./feed_audio.sh default (captures PulseAudio default source)"
    exit 1
fi

echo "Feeding audio from $SOURCE to $AUDIO_PATH..."
echo "Format: s16le, 44100Hz, Stereo"

if [[ "$SOURCE" == "default" ]]; then
    # Capture from PulseAudio and convert to raw PCM
    ffmpeg -f pulse -i default -f s16le -ar 44100 -ac 2 -y "$AUDIO_PATH"
else
    # Read from file/URL and convert to raw PCM
    ffmpeg -i "$SOURCE" -f s16le -ar 44100 -ac 2 -y "$AUDIO_PATH"
fi
