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

#ifndef BOB_SDLVIZ__SDLVIZ_HPP_
#define BOB_SDLVIZ__SDLVIZ_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bob_sdlviz/terminal.hpp"
#include "bob_sdlviz/types.hpp"

class SdlVizNode : public rclcpp::Node
{
public:
  SdlVizNode();
  ~SdlVizNode();

  void run();

private:
  void event_callback(const std_msgs::msg::String::SharedPtr msg);
  void process_terminal_config(const std::string & json_data);
  void render_loop();

  // ROS 2 components
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  rclcpp::CallbackGroup::SharedPtr render_group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_subscriber_;

  // Threading and Data
  std::mutex data_mutex_;
  std::map<std::string, std::unique_ptr<DynamicMarkerLayer>>
  dynamic_marker_layers_;
  std::mutex marker_layers_mutex_;
  std::string last_string_;

  // SDL and Rendering
  SDL_Window * window_ = nullptr;
  SDL_Renderer * renderer_ = nullptr;
  SDL_Surface * surface_ = nullptr;  // For off-screen rendering
  TTF_Font * font_ = nullptr;
  int screen_width_ = 854;
  int screen_height_ = 480;
  bool show_window_ = true;
  bool stream_output_ = false;

  // Dynamic Content
  std::map<std::string, std::unique_ptr<DynamicTerminal>> dynamic_terminals_;
  std::mutex terminals_mutex_;
  std::map<std::string, std::unique_ptr<DynamicVideoStream>>
  dynamic_video_streams_;
  std::mutex video_streams_mutex_;
  std::vector<uint8_t> video_frame_buffer_;

  // Output
  std::unique_ptr<FifoStreamer> streamer_;
};

#endif  // BOB_SDLVIZ__SDLVIZ_HPP_
