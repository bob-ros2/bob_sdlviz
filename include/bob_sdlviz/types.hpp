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

// A simple helper class for streaming raw frames to a named pipe (FIFO)
class FifoStreamer
{
public:
  explicit FifoStreamer(const std::string & path)
  : pipe_path_(path) {}

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

// A helper for reading raw frames from a FIFO (e.g. from an MPV instance)
class FifoReader
{
public:
  FifoReader(const std::string & path, int w, int h)
  : path_(path), width_(w), height_(h)
  {
    frame_size_ = width_ * height_ * 4;  // BGRA
    current_frame_.resize(frame_size_, 0);
  }

  void start()
  {
    reader_thread_ = std::thread(
      [this]() {
        while (!stop_flag_) {
          int fd = open(path_.c_str(), O_RDONLY);
          if (fd == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
          }

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
        }
      });
  }

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

struct DynamicTerminal
{
  std::unique_ptr<yTerminal> terminal;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
  rclcpp::Time creation_time;
  rclcpp::Duration lifetime;

  DynamicTerminal()
  : creation_time(0, 0, RCL_ROS_TIME), lifetime(0, 0) {}
};

struct DynamicVideoStream
{
  std::unique_ptr<FifoReader> reader;
  SDL_Texture * texture = nullptr;
  SDL_Rect area;
  int source_width;
  int source_height;

  ~DynamicVideoStream()
  {
    if (texture) {
      SDL_DestroyTexture(texture);
    }
  }
};

struct DynamicMarkerLayer
{
  SDL_Rect area;
  double scale;
  double offset_x;
  double offset_y;
  std::vector<std::string> excluded_ns;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
    subscriber;
  visualization_msgs::msg::MarkerArray::SharedPtr last_markers;
  std::mutex data_mutex;
};

#endif  // BOB_SDLVIZ__TYPES_HPP_
