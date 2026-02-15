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
 * @file sdlviz.hpp
 * @brief Primary ROS 2 node class for SDL-based visualization.
 *
 * This file defines the SdlVizNode class, which integrates ROS 2 subscriptions
 * with an SDL2 rendering loop for markers, terminals, and video streams.
 */

#ifndef BOB_SDLVIZ__SDLVIZ_HPP_
#define BOB_SDLVIZ__SDLVIZ_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bob_sdlviz/terminal.hpp"
#include "bob_sdlviz/types.hpp"

/**
 * @class SdlVizNode
 * @brief Main ROS 2 node for the bob_sdlviz package.
 *
 * Handles orchestration of dynamic layers, window management, and frame streaming.
 */
class SdlVizNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new SdlVizNode object.
   *
   * Initializes parameters, sources environment variables, and sets up SDL.
   */
  SdlVizNode();

  /**
   * @brief Clean up SDL resources.
   */
  ~SdlVizNode();

  /**
   * @brief Main execution entry point. Runs the SDL event and render loop.
   */
  void run();

private:
  /**
   * @brief Callback for the dynamic configuration events topic.
   * @param msg The JSON string containing layer configurations.
   */
  void event_callback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Parses and applies a terminal configuration JSON.
   * @param json_data The raw JSON string.
   */
  void process_terminal_config(const std::string & json_data);

  /**
   * @brief Internal rendering loop called by run(). Distributes draw calls to layers.
   */
  void render_loop();

  // ROS 2 components
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;      ///< Group for event callbacks.
  rclcpp::CallbackGroup::SharedPtr render_group_;         ///< Group for the main rendering loop.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_subscriber_;  ///< Listener.

  // Threading and Data
  std::mutex data_mutex_;                                 ///< Global data protection.
  std::map<std::string, std::unique_ptr<DynamicMarkerLayer>>
  dynamic_marker_layers_;  ///< Map of active marker layers.
  std::mutex marker_layers_mutex_;                        ///< Protects the marker layers map.
  std::string last_string_;                               ///< Cache for last string data.

  // SDL and Rendering
  SDL_Window * window_ = nullptr;                         ///< Local window handle.
  SDL_Renderer * renderer_ = nullptr;                     ///< SDL renderer handle.
  SDL_Surface * surface_ = nullptr;                       ///< Surface for headless streaming.
  TTF_Font * font_ = nullptr;                             ///< Default TTF font handle.
  int screen_width_ = 854;                                ///< Final output width.
  int screen_height_ = 480;                               ///< Final output height.
  bool show_window_ = true;                               ///< Toggle for local UI.
  bool stream_output_ = false;                            ///< Toggle for FIFO streaming.

  // Dynamic Content
  std::map<std::string, std::unique_ptr<DynamicTerminal>> dynamic_terminals_;  ///< Text areas.
  std::mutex terminals_mutex_;                            ///< Protects terminal areas.
  std::map<std::string, std::unique_ptr<DynamicVideoStream>>
  dynamic_video_streams_;                                 ///< Map of video overlays.
  std::mutex video_streams_mutex_;                        ///< Protects video streams.
  std::vector<uint8_t> video_frame_buffer_;               ///< Buffer for frame acquisition.

  // Output
  std::unique_ptr<FifoStreamer> streamer_;                ///< Helper for FIFO frame output.
};

#endif  // BOB_SDLVIZ__SDLVIZ_HPP_
