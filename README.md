# bob_sdlviz

A high-performance ROS 2 visualization node based on SDL2. Designed for flexible, headless streaming of markers, video frames, and dynamic text overlays to platforms like Twitch.

## Table of Contents
- [Overview](#overview)
- [Installation](#installation)
- [Building](#building)
- [Usage](#usage)
- [ROS 2 API](#ros-2-api)
    - [Parameters](#parameters)
    - [Topics](#topics)
- [Dynamic Configuration](#dynamic-configuration)
- [Audio System](#audio-system)
- [Docker Deployment](#docker-deployment)
    - [Dockerfile](#dockerfile)
    - [Docker Compose](#docker-compose)
- [Streaming & Headless Operation](#streaming--headless-operation)

---

## Overview

`bob_sdlviz` allows you to create complex visualization layouts combining:
- **Marker Layers**: Render `visualization_msgs/MarkerArray` with custom scaling, offsets, and namespace filtering.
- **Terminal Layers**: Dynamic text overlays with word-wrapping and auto-expiration.
- **Video Streams**: Integration of raw BGRA frame buffers from external sources (e.g., FFmpeg/MPV).

It is specifically optimized for **Docker** and **headless streaming** using a dummy video driver.

---

## Installation

### Dependencies
Ensure you have the following system libraries installed:
```bash
sudo apt update
sudo apt install libsdl2-dev libsdl2-ttf-dev nlohmann-json3-dev
```

### ROS 2 Dependencies
The package requires standard ROS 2 message libraries:
- `rclcpp`
- `std_msgs`
- `visualization_msgs`

---

## Building

Standard colcon build:
```bash
cd ~/ros2_ws
colcon build --packages-select bob_sdlviz
source install/setup.bash
```

---

## Usage

### Launching
Run the node directly:
```bash
ros2 run bob_sdlviz sdlviz
```

### Configuration via Environment Variables
Many parameters can be mapped from environment variables for easier Docker integration:
| Env Variable | Used for Parameter | Default |
|--------------|--------------------|---------|
| `SDLVIZ_WIDTH` | `screen_width` | `854` |
| `SDLVIZ_HEIGHT` | `screen_height` | `480` |
| `SDLVIZ_SHOW_WINDOW`| `show_window` | `true` |
| `SDLVIZ_STREAM_OUTPUT`| `stream_output`| `false` |
| `SDLVIZ_STREAM_PATH`| `stream_path` | `/tmp/video_pipe` |
| `SDLVIZ_CONFIG_PATH`| `config_file_path`| `""` |
| `SDLVIZ_FONT_PATH` | `font_path` | `/usr/share/fonts/...` |
| `SDLVIZ_FPS` | `fps` | `30.0` |

---

## ROS 2 API

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `screen_width` | `int` | Target screen/video width in pixels. (Env: `SDLVIZ_WIDTH`) |
| `screen_height` | `int` | Target screen/video height in pixels. (Env: `SDLVIZ_HEIGHT`) |
| `show_window` | `bool` | Whether to show the local SDL window. (Env: `SDLVIZ_SHOW_WINDOW`) |
| `stream_output` | `bool` | Enable writing frames to a FIFO pipe. (Env: `SDLVIZ_STREAM_OUTPUT`) |
| `stream_path` | `string` | Path to the output FIFO pipe for streaming. (Env: `SDLVIZ_STREAM_PATH`) |
| `config_file_path`| `string` | Path to a JSON file for initial layout. (Env: `SDLVIZ_CONFIG_PATH`) |
| `font_path` | `string` | Path to the TTF font file. (Env: `SDLVIZ_FONT_PATH`) |
| `font_size` | `int` | Base font size for terminals. (Env: `SDLVIZ_FONT_SIZE`) |
| `fps` | `double` | Target rendering and streaming FPS. (Env: `SDLVIZ_FPS`) |

### Topics

#### Subscribed
- `events` (`std_msgs/msg/String`): Primary control topic. Receives JSON arrays to define dynamic layers.
- **Dynamic Topics**: Subscribes to topics defined in the JSON configuration (e.g., marker topics or text strings).

---

## Dynamic Configuration

The node is controlled by sending JSON arrays to the `events` topic.

### Layer Types
- **`String`**: Creates a text terminal area.
- **`MarkerLayer`**: Creates a localized 2D marker rendering layer.
- **`VideoStream`**: Creates an area that reads raw frames from a FIFO.

### Video Stream Integration

To feed an external video source into `sdlviz` node:

#### 1. General FIFO streaming
1. **Create a FIFO pipe**:
   ```bash
   mkfifo /tmp/overlay_video
   ```

2. **Feed the pipe** with FFmpeg (BGRA format at real-time speed):
   ```bash
   ffmpeg -re -f lavfi -i testsrc=size=854x480:rate=30 -f rawvideo -pix_fmt bgra /tmp/overlay_video
   ```

#### 2. Streaming a Terminal Window (Linux/X11)
To capture a specific terminal window and pipe it into the Docker container:

1. **Find window geometry**: Run `xwininfo` and click on the target terminal. Note the `-geometry` line (e.g., `854x480+10+10`).
2. **Stream to container**:
   ```bash
   # Use the width, height, and offsets from xwininfo.
   # The write_fifo.sh script inside the container creates the FIFO and handles the input.
   ffmpeg -re -f x11grab -video_size 854x480 -i :0.0+10,10 -f rawvideo -pix_fmt bgra - | \
     docker exec -i nexus_streamer /root/ros2_ws/install/bob_sdlviz/lib/bob_sdlviz/write_fifo.sh --path /tmp/overlay_video
   ```

#### 3. Spawn the layer
Send JSON to the `/events` topic:
```json
{
  "type": "VideoStream",
  "topic": "/tmp/overlay_video",
  "area": [220, 10, 414, 300],
  "source_width": 854,
  "source_height": 480,
  "expire": 0.0
}
```

> [!NOTE]
> `sdlviz` expects exactly **4 bytes per pixel (BGRA)**. Using 3-byte formats (like RGB or BGR) will result in distorted images.

### Example JSON
```json
[
  {
    "type": "String",
    "topic": "/bob/log",
    "area": [10, 10, 400, 200],
    "text_color": [255, 255, 255, 255],
    "bg_color": [0, 0, 0, 150]
  },
  {
    "type": "MarkerLayer",
    "topic": "/bob/markers",
    "area": [420, 10, 400, 400],
    "scale": 1500.0,
    "exclude_ns": "background"
  }
]
```

---

## Audio System

`bob_sdlviz` implements a robust, non-blocking audio architecture to ensure streams never stall and audio remains synchronized.

### Non-Blocking Audio Bridge
The `audio_bridge.py` script acts as a persistent master source for FFmpeg. It:
1. **Silence Injection**: Continuously feeds silent audio frames into a master pipe (`/tmp/audio_master_pipe`).
2. **Dynamic Mixing**: Listens to the user-facing audio pipe (`/tmp/audio_pipe`) and mixes any incoming audio with the baseline silence.

This ensures that FFmpeg always has an active audio stream, even if you stop or start your external audio source.

### Feeding Audio
To inject audio into the running stream from your host machine, use the `feed_audio.sh` script:
```bash
# Feed a music file
./scripts/feed_audio.sh my_music.mp3

# Feed local system audio (PulseAudio)
./scripts/feed_audio.sh default
```

### Configuration
| Env Variable | Description |
|--------------|-------------|
| `ENABLE_AUDIO` | `fifo` (Use the bridge), `pulse` (Direct host audio), or `false`. |
| `SDLVIZ_AUDIO_PATH` | The user-facing pipe to write audio into (`/tmp/audio_pipe`). |
| `SDLVIZ_AUDIO_MASTER_PATH` | The internal pipe used by the bridge (`/tmp/audio_master_pipe`). |

---

## Docker Deployment

`bob_sdlviz` is designed to be fully containerized, allowing for consistent streaming environments without host-side dependencies (like X-Server).

### .env Configuration
Copy the provided template to create your own `.env` file:
```bash
cp .env.template .env
```
Ensure you set your `TWITCH_STREAM_KEY` in this file. This key is used by `start_stream.sh` to authenticate with Twitch.

### Dockerfile
The included `Dockerfile` builds a complete ROS 2 Humble environment including:
- **SDL2 & TTF**: For high-quality text and marker rendering.
- **FFmpeg**: For RTMP streaming.
- **Python Bridge**: For silence injection and audio mixing.

### Docker Compose
Use Docker Compose for the easiest deployment. It handles volume mounting for the Unix pipes and sets the necessary environment variables.

**To start the streamer:**
```bash
docker compose up --build
```
*Note: The container runs with `network_mode: host` to allow seamless discovery of ROS topics on your local network.*

---

## Streaming & Headless Operation

To run in a container or on a remote server without a GPU or X-Server, follow these critical steps:

### 1. Enable Dummy Driver
Set the environment variable `SDL_VIDEODRIVER=dummy`. This tells SDL to perform all rendering in software memory rather than attempting to open a graphical window.

### 2. Configure for Pipe Output
Ensure the following parameters are set (either in `.env` or as ROS parameters):
- `show_window`: `false`
- `stream_output`: `true`
- `stream_path`: `/tmp/video_pipe` (Matches the volume mount in Docker Compose)

### 3. FFMPEG Orchestration
The primary entrypoint in the container (`start_stream.sh`) automatically coordinates:
- Starting the `sdlviz` node.
- Starting the `audio_bridge.py` to provide a baseline silent audio stream.
- Launching `ffmpeg` to combine the video pipe and audio pipe into a single FLV stream sent to Twitch.

- **INGEST_SERVER Configuration**: You can change the Twitch ingest server (e.g., for different regions) by setting the `INGEST_SERVER` environment variable (Default: Frankfurt, DE).
- **Headless Mode Enforcement**: The container defaults to `SDL_VIDEODRIVER=dummy`, ensuring that no physical or virtual display is required.

**Manual FFMPEG Command Example:**
If you wish to run FFMPEG manually (e.g., outside of Docker), use this optimized configuration for low-latency streaming:
```bash
ffmpeg -f rawvideo -pixel_format bgra -video_size 854x480 -framerate 30 -i /tmp/video_pipe \
       -f s16le -ar 44100 -ac 2 -i /tmp/audio_pipe \
       -vcodec libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p \
       -g 60 -b:v 3000k -maxrate 3000k -bufsize 6000k \
       -acodec aac -ab 128k -f flv "${INGEST_SERVER}${TWITCH_STREAM_KEY}"
```
