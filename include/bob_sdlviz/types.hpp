// Copyright 2026 Bob Ros
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file types.hpp
 * @brief Common data structures and FIFO utility classes for bob_sdlviz.
 *
 * This file defines the core structures used for dynamic marker layers,
 * terminals, and video streams, as well as helpers for pipe-based I/O.
 */

#ifndef BOB_SDLVIZ__TYPES_HPP_
#define BOB_SDLVIZ__TYPES_HPP_

#include <fcntl.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Forward declaration of yTerminal to avoid circular dependency
class yTerminal;

/**
 * @class FifoStreamer
 * @brief A simple helper class for streaming raw frames to a named pipe (FIFO).
 */
class FifoStreamer
{
public:
  /**
   * @brief Construct a new FifoStreamer object.
   * @param path The filesystem path to the named pipe.
   */
  explicit FifoStreamer(const std::string & path)
  : pipe_path_(path) {}

  /**
   * @brief Opens the pipe for writing. Creates the FIFO if it doesn't exist.
   * @return true if successfully opened, false otherwise.
   */
  bool open_pipe()
  {
    if (access(pipe_path_.c_str(), F_OK) == -1) {
      if (mkfifo(pipe_path_.c_str(), 0666) == -1) {
        perror("mkfifo");
      }
    }

    pipe_fd_ = open(pipe_path_.c_str(), O_WRONLY);
    if (pipe_fd_ != -1) {
      if (fcntl(pipe_fd_, 1031, 4 * 1024 * 1024) == -1) {
        // Optimization failed, ignore
      }
      return true;
    }
    return false;
  }

  /**
   * @brief Writes a frame buffer to the pipe. Automatically opens the pipe on first call.
   * @param buffer Pointer to the frame data.
   * @param size Size of the data in bytes.
   */
  void write_frame(void * buffer, size_t size)
  {
    if (pipe_fd_ == -1) {
      if (!open_pipe()) {
        return;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("sdlviz"),
        "Output FIFO opened for writing.");
    }
    if (write(pipe_fd_, buffer, size) == -1) {
      if (errno == EPIPE) {
        RCLCPP_WARN(rclcpp::get_logger("sdlviz"), "Pipe broken, closing.");
        close(pipe_fd_);
        pipe_fd_ = -1;
      }
    }
  }

  /**
   * @brief Destroy the FifoStreamer object and close the pipe.
   */
  ~FifoStreamer()
  {
    if (pipe_fd_ != -1) {
      close(pipe_fd_);
    }
  }

private:
  std::string pipe_path_;
  int pipe_fd_ = -1;
};

/**
 * @class FifoReader
 * @brief A helper for reading raw frames from a FIFO (e.g. from an MPV or FFmpeg instance).
 */
class FifoReader
{
public:
  /**
   * @brief Construct a new FifoReader object.
   * @param path Path to the named pipe to read from.
   * @param w Expected frame width.
   * @param h Expected frame height.
   */
  FifoReader(const std::string & path, int w, int h)
  : path_(path), width_(w), height_(h)
  {
    frame_size_ = width_ * height_ * 4;  // BGRA
    current_frame_.resize(frame_size_, 0);
  }

  /**
   * @brief Starts the background reader thread.
   *
   * Opens the FIFO non-blockingly (to avoid hanging if no writer is present),
   * then switches to blocking mode for reliable frame reads. Automatically
   * reconnects if the producer disconnects or has not yet started.
   */
  void start()
  {
    reader_thread_ = std::thread(
      [this]() {
        while (!stop_flag_) {
          // O_NONBLOCK: don't hang if webvideo isn't running yet
          int fd = open(path_.c_str(), O_RDONLY | O_NONBLOCK);
          if (fd == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
          }
          // Switch back to blocking for reliable full-frame reads
          int flags = fcntl(fd, F_GETFL, 0);
          fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

          RCLCPP_INFO(
            rclcpp::get_logger("sdlviz"),
            "FIFO video stream connected: %s", path_.c_str());

          std::vector<uint8_t> buffer(frame_size_);
          while (!stop_flag_) {
            size_t bytes_read = 0;
            while (bytes_read < frame_size_ && !stop_flag_) {
              ssize_t r =
              read(fd, buffer.data() + bytes_read, frame_size_ - bytes_read);
              if (r <= 0) {
                break;
              }
              bytes_read += r;
            }

            if (bytes_read == frame_size_) {
              std::lock_guard<std::mutex> lock(frame_mutex_);
              current_frame_ = buffer;
              new_frame_available_ = true;
            } else {
              break;
            }
          }
          close(fd);
          RCLCPP_INFO(
            rclcpp::get_logger("sdlviz"),
            "FIFO video stream disconnected: %s (reconnecting...)",
            path_.c_str());
        }
      });
  }

  /**
   * @brief Fetches the latest frame if available.
   * @param[out] out_frame Buffer to copy the frame into.
   * @return true if a new frame was copied, false otherwise.
   */
  bool get_latest_frame(std::vector<uint8_t> & out_frame)
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!new_frame_available_) {
      return false;
    }
    out_frame = current_frame_;
    new_frame_available_ = false;
    return true;
  }

  /**
   * @brief Destroy the FifoReader object and stop the reader thread.
   */
  ~FifoReader()
  {
    stop_flag_ = true;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

private:
  std::string path_;
  int width_, height_;
  size_t frame_size_;
  std::thread reader_thread_;
  std::atomic<bool> stop_flag_{false};
  std::mutex frame_mutex_;
  std::vector<uint8_t> current_frame_;
  bool new_frame_available_{false};
};

/**
 * @struct DynamicTerminal
 * @brief Represents a dynamically created text terminal area.
 */
struct DynamicTerminal
{
  std::unique_ptr<yTerminal> terminal;  ///< The terminal rendering logic.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;  ///< Subscription.
  rclcpp::Time creation_time;           ///< When the layer was created.
  rclcpp::Duration lifetime;            ///< Expiration duration (0 = infinite).

  DynamicTerminal()
  : creation_time(0, 0, RCL_ROS_TIME), lifetime(0, 0) {}
};

/**
 * @struct DynamicVideoStream
 * @brief Represents a dynamic video area reading frames from a FIFO.
 */
struct DynamicVideoStream
{
  std::unique_ptr<FifoReader> reader;  ///< The FIFO reader instance.
  SDL_Texture * texture = nullptr;     ///< SDL texture for rendering.
  SDL_Rect area;                       ///< Screen area to render into.
  int source_width;                    ///< Raw frame width.
  int source_height;                   ///< Raw frame height.
  rclcpp::Time creation_time;           ///< When the layer was created.
  rclcpp::Duration lifetime;            ///< Expiration duration (0 = infinite).

  DynamicVideoStream()
  : creation_time(0, 0, RCL_ROS_TIME), lifetime(0, 0) {}

  ~DynamicVideoStream()
  {
    if (texture) {
      SDL_DestroyTexture(texture);
    }
  }
};

/**
 * @struct DynamicMarkerLayer
 * @brief Represents a 2D projection layer for MarkerArrays.
 */
struct DynamicMarkerLayer
{
  SDL_Rect area;         ///< Screen area to render markers within.
  double scale;          ///< Scaling factor for marker coordinates.
  double offset_x;       ///< Horizontal offset.
  double offset_y;       ///< Vertical offset.
  std::vector<std::string> excluded_ns;  ///< Namespaces to ignore in this layer.
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
    subscriber;          ///< MarkerArray topic subscription.
  visualization_msgs::msg::MarkerArray::SharedPtr last_markers;  ///< Latest received markers.
  std::mutex data_mutex;  ///< Mutex for protecting last_markers.
  rclcpp::Time creation_time;           ///< When the layer was created.
  rclcpp::Duration lifetime;            ///< Expiration duration (0 = infinite).

  DynamicMarkerLayer()
  : creation_time(0, 0, RCL_ROS_TIME), lifetime(0, 0) {}
};

#endif  // BOB_SDLVIZ__TYPES_HPP_
