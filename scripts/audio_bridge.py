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
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import threading
import collections

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


class AudioMixer(Node):
    def __init__(self):
        super().__init__('audio_bridge')
        self.input_path = os.environ.get('SDLVIZ_AUDIO_PATH', '/tmp/audio_pipe')
        self.master_path = os.environ.get('SDLVIZ_AUDIO_MASTER_PATH', '/tmp/audio_master_pipe')

        # Buffer for incoming ROS audio (e.g. from TTS)
        # Expected format: s16le, 44100Hz, Mono or Stereo
        self.tts_buffer = collections.deque()
        self.lock = threading.Lock()

        self.sub = self.create_subscription(
            Int16MultiArray,
            'audio_raw',
            self.audio_callback,
            10
        )
        self.get_logger().info("Audio Bridge initialized. Subscribed to 'audio_raw'.")

    def audio_callback(self, msg):
        """Handle incoming raw audio (expected Int16, 44100Hz)."""
        data = np.array(msg.data, dtype=np.int16)

        # If it's mono (1 channel), convert to stereo by duplicating
        # We don't have metadata here, but we can guess based on expected frame size
        # or just assume mono and convert.
        # For simplicity, we assume incoming is mono 44.1kHz.
        stereo_data = np.stack((data, data), axis=-1).flatten()

        with self.lock:
            # Add to buffer
            self.tts_buffer.extend(stereo_data.tolist())

    def get_tts_chunk(self, num_samples):
        """Extract a chunk of samples from the TTS buffer."""
        with self.lock:
            if len(self.tts_buffer) < num_samples:
                chunk = list(self.tts_buffer)
                self.tts_buffer.clear()
                # Pad with zeros
                chunk.extend([0] * (num_samples - len(chunk)))
                return np.array(chunk, dtype=np.int16)
            else:
                chunk = [self.tts_buffer.popleft() for _ in range(num_samples)]
                return np.array(chunk, dtype=np.int16)


def main():
    rclpy.init()
    mixer = AudioMixer()

    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(mixer,), daemon=True)
    ros_thread.start()

    print(f"Audio Bridge: {mixer.input_path} -> {mixer.master_path}")

    # Ensure master pipe exists
    if not os.path.exists(mixer.master_path):
        os.mkfifo(mixer.master_path)

    # Open master for writing
    print("Waiting for FFmpeg to open master audio pipe...")
    master = open(mixer.master_path, 'wb')
    print("Master audio pipe opened.")

    # Open input pipe (sdlviz game audio)
    try:
        infd = os.open(mixer.input_path, os.O_RDWR | os.O_NONBLOCK)
        print(f"Input audio pipe {mixer.input_path} opened (Keep-Alive mode).")
    except Exception as e:
        print(f"Error opening input pipe: {e}")
        return

    last_time = time.time()

    try:
        while rclpy.ok():
            game_data_raw = b''
            try:
                game_data_raw = os.read(infd, CHUNK_BYTES)
            except (BlockingIOError, OSError):
                pass

            # Convert game audio to numpy
            if game_data_raw:
                if len(game_data_raw) < CHUNK_BYTES:
                    game_data_raw += b'\x00' * (CHUNK_BYTES - len(game_data_raw))
                game_audio = np.frombuffer(game_data_raw, dtype=np.int16)
            else:
                game_audio = np.zeros(CHUNK_FRAMES * CHANNELS, dtype=np.int16)

            # Get TTS audio
            tts_audio = mixer.get_tts_chunk(CHUNK_FRAMES * CHANNELS)

            # Mix (Add)
            # Use int32 for mixing to avoid overflow before clipping back to int16
            mixed_audio = game_audio.astype(np.int32) + tts_audio.astype(np.int32)
            mixed_audio = np.clip(mixed_audio, -32768, 32767).astype(np.int16)

            # Write to master
            master.write(mixed_audio.tobytes())
            master.flush()

            # Timing
            current_time = time.time()
            elapsed = current_time - last_time
            sleep_time = CHUNK_DURATION - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_time = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        os.close(infd)
        master.close()
        rclpy.shutdown()


if __name__ == "__main__":
    import numpy as np  # Ensure numpy is imported for main
    main()
