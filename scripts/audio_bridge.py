#!/usr/bin/env python3
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
import os
import time

# Audio parameters: s16le, 44100Hz, Stereo
# 4 bytes per frame (2 bytes per sample * 2 channels)
RATE = 44100
CHANNELS = 2
SAMPLE_SIZE = 2
FRAME_SIZE = CHANNELS * SAMPLE_SIZE

# We feed in chunks of 20ms
CHUNK_DURATION = 0.02
CHUNK_FRAMES = int(RATE * CHUNK_DURATION)
CHUNK_BYTES = CHUNK_FRAMES * FRAME_SIZE
SILENCE = b'\x00' * CHUNK_BYTES


def main():
    input_path = os.environ.get('SDLVIZ_AUDIO_PATH', '/tmp/audio_pipe')
    master_path = os.environ.get('SDLVIZ_AUDIO_MASTER_PATH', '/tmp/audio_master_pipe')

    print(f"Audio Bridge: {input_path} -> {master_path}")

    # Ensure master pipe exists
    if not os.path.exists(master_path):
        os.mkfifo(master_path)

    # Open master for writing. This will block until the reader (ffmpeg) opens it.
    print("Waiting for FFmpeg to open master audio pipe...")
    master = open(master_path, 'wb')
    print("Master audio pipe opened.")

    # Open input pipe in R/W mode to keep it always "open" even without writers.
    # This prevents writers from getting "Broken Pipe" and prevents us from seeing EOF.
    try:
        infd = os.open(input_path, os.O_RDWR | os.O_NONBLOCK)
        print(f"Input audio pipe {input_path} opened (Keep-Alive mode).")
    except Exception as e:
        print(f"Error opening input pipe: {e}")
        return

    last_time = time.time()

    try:
        while True:
            data = b''
            try:
                data = os.read(infd, CHUNK_BYTES)
            except (BlockingIOError, OSError):
                pass

            # Note: We won't see EOF (data == b'') because we hold a writer FD (O_RDWR).
            # This is exactly what we want to remain "non-blocking" and "always readable".

            # Fill or Pad
            if not data:
                master.write(SILENCE)
            else:
                if len(data) < CHUNK_BYTES:
                    data += b'\x00' * (CHUNK_BYTES - len(data))
                master.write(data)

            master.flush()

            # Timing: We want to maintain real-time flow.
            # FFmpeg's consumption from the FIFO will provide backpressure,
            # but we also sleep slightly to not spin if the FIFO doesn't block.
            current_time = time.time()
            elapsed = current_time - last_time
            sleep_time = CHUNK_DURATION - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_time = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        if infd is not None:
            os.close(infd)
        master.close()


if __name__ == "__main__":
    main()
