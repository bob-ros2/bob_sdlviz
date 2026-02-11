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

#include "bob_sdlviz/sdlviz.hpp"
#include <algorithm>
#include <cctype>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

using namespace std::chrono_literals;

// Helper functions to get environment variables with defaults
static std::string get_env(const char * name, const std::string & default_val)
{
  const char * val = std::getenv(name);
  return val ? std::string(val) : default_val;
}

static int get_env(const char * name, int default_val)
{
  const char * val = std::getenv(name);
  if (!val) {
    return default_val;
  }
  try {
    return std::stoi(val);
  } catch (...) {
    return default_val;
  }
}

static double get_env(const char * name, double default_val)
{
  const char * val = std::getenv(name);
  if (!val) {
    return default_val;
  }
  try {
    return std::stod(val);
  } catch (...) {
    return default_val;
  }
}

static bool get_env(const char * name, bool default_val)
{
  const char * val = std::getenv(name);
  if (!val) {
    return default_val;
  }
  std::string s(val);
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return std::tolower(c);});
  return s == "true" || s == "1" || s == "yes" || s == "on";
}

static std::vector<std::string>
get_env(const char * name, const std::vector<std::string> & default_val)
{
  const char * val = std::getenv(name);
  if (!val) {
    return default_val;
  }
  std::vector<std::string> result;
  std::stringstream ss(val);
  std::string item;
  while (std::getline(ss, item, ',')) {
    result.push_back(item);
  }
  return result;
}

SdlVizNode::SdlVizNode()
: Node("sdlviz")
{
  signal(SIGPIPE, SIG_IGN);

  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

  descriptor.description =
    "Uniform scale factor for all markers (Default: 1200.0, Env: SDLVIZ_SCALE).";
  this->declare_parameter("scale", get_env("SDLVIZ_SCALE", 1200.0), descriptor);

  descriptor.description =
    "Horizontal pixel offset for markers (Default: -50.0, Env: SDLVIZ_OFFSET_X).";
  this->declare_parameter(
    "offset_x", get_env("SDLVIZ_OFFSET_X", -50.0),
    descriptor);

  descriptor.description =
    "Vertical pixel offset for markers (Default: -110.0, Env: SDLVIZ_OFFSET_Y).";
  this->declare_parameter(
    "offset_y", get_env("SDLVIZ_OFFSET_Y", -110.0),
    descriptor);

  descriptor.description =
    "Target screen/video width in pixels (Default: 854, Env: SDLVIZ_WIDTH).";
  this->declare_parameter(
    "screen_width", get_env("SDLVIZ_WIDTH", 854),
    descriptor);

  descriptor.description =
    "Target screen/video height in pixels (Default: 480, Env: SDLVIZ_HEIGHT).";
  this->declare_parameter(
    "screen_height", get_env("SDLVIZ_HEIGHT", 480),
    descriptor);

  descriptor.description =
    "Whether to show the local SDL visualization window "
    "(Default: true, Env: SDLVIZ_SHOW_WINDOW).";
  this->declare_parameter(
    "show_window", get_env("SDLVIZ_SHOW_WINDOW", true),
    descriptor);

  descriptor.description =
    "Enable writing raw BGRA frames to a FIFO pipe "
    "(Default: false, Env: SDLVIZ_STREAM_OUTPUT).";
  this->declare_parameter(
    "stream_output",
    get_env("SDLVIZ_STREAM_OUTPUT", false), descriptor);

  descriptor.description =
    "Path to the output FIFO pipe for streaming "
    "(Default: /tmp/video_pipe, Env: SDLVIZ_STREAM_PATH).";
  this->declare_parameter(
    "stream_path",
    get_env("SDLVIZ_STREAM_PATH", std::string("/tmp/video_pipe")),
    descriptor);

  descriptor.description =
    "Path to a JSON file defining initial terminal layouts "
    "(Default: '', Env: SDLVIZ_CONFIG_PATH).";
  this->declare_parameter(
    "config_file_path",
    get_env("SDLVIZ_CONFIG_PATH", std::string("")),
    descriptor);

  descriptor.description =
    "Path to the TTF font file used for text rendering "
    "(Default: DejaVuSans, Env: SDLVIZ_FONT_PATH).";
  this->declare_parameter(
    "font_path",
    get_env(
      "SDLVIZ_FONT_PATH",
      std::string("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")),
    descriptor);

  descriptor.description =
    "Base font size for terminal text rendering "
    "(Default: 16, Env: SDLVIZ_FONT_SIZE).";
  this->declare_parameter(
    "font_size", get_env("SDLVIZ_FONT_SIZE", 16),
    descriptor);

  descriptor.description =
    "Target rendering and streaming frames per second "
    "(Default: 30.0, Env: SDLVIZ_FPS).";
  this->declare_parameter("fps", get_env("SDLVIZ_FPS", 30.0), descriptor);

  descriptor.description =
    "List of marker namespaces to exclude from rendering "
    "(Default: [], Env: SDLVIZ_EXCLUDE_NS).";
  this->declare_parameter(
    "excluded_namespaces",
    get_env("SDLVIZ_EXCLUDE_NS", std::vector<std::string>{}), descriptor);

  reentrant_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  render_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  show_window_ = this->get_parameter("show_window").as_bool();
  stream_output_ = this->get_parameter("stream_output").as_bool();
  screen_width_ = this->get_parameter("screen_width").as_int();
  screen_height_ = this->get_parameter("screen_height").as_int();

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "SDL could not initialize! SDL_Error: %s",
      SDL_GetError());
    return;
  }

  if (TTF_Init() == -1) {
    RCLCPP_ERROR(
      this->get_logger(),
      "SDL_ttf could not initialize! TTF_Error: %s", TTF_GetError());
    return;
  }

  auto font_path = this->get_parameter("font_path").as_string();
  auto font_size = this->get_parameter("font_size").as_int();
  font_ = TTF_OpenFont(font_path.c_str(), font_size);
  if (!font_) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load font: %s! TTF_Error: %s",
      font_path.c_str(), TTF_GetError());
    return;
  }

  if (show_window_) {
    RCLCPP_INFO(
      this->get_logger(), "Creating window (%dx%d)...", screen_width_,
      screen_height_);
    window_ = SDL_CreateWindow(
      "SdlViz", SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED, screen_width_,
      screen_height_, SDL_WINDOW_SHOWN);
    if (!window_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Window could not be created! SDL_Error: %s",
        SDL_GetError());
      return;
    }
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to create accelerated renderer: %s. Falling back.",
        SDL_GetError());
      renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
    }
    if (renderer_) {
      SDL_RendererInfo info;
      if (SDL_GetRendererInfo(renderer_, &info) == 0) {
        RCLCPP_INFO(
          this->get_logger(), "Renderer: %s (HW:%s, VSync:%s)",
          info.name,
          (info.flags & SDL_RENDERER_ACCELERATED) ? "Yes" : "No",
          (info.flags & SDL_RENDERER_PRESENTVSYNC) ? "Yes" : "No");
      }
    }
  } else {
    surface_ =
      SDL_CreateRGBSurface(
      0, screen_width_, screen_height_, 32, 0x00ff0000,
      0x0000ff00, 0x000000ff, 0xff000000);
    if (!surface_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Surface could not be created! SDL_Error: %s",
        SDL_GetError());
      return;
    }
    renderer_ = SDL_CreateSoftwareRenderer(surface_);
    if (!renderer_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Software renderer could not be created! SDL_Error: %s",
        SDL_GetError());
      return;
    }
  }

  if (stream_output_) {
    std::string stream_path = this->get_parameter("stream_path").as_string();
    streamer_ = std::make_unique<FifoStreamer>(stream_path);
    if (!streamer_->open_pipe()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Stream pipe at %s not ready yet (waiting for reader).",
        stream_path.c_str());
    }
  }

  auto config_path = this->get_parameter("config_file_path").as_string();
  if (!config_path.empty()) {
    std::ifstream config_file(config_path);
    if (config_file.is_open()) {
      RCLCPP_INFO(
        this->get_logger(), "Loading terminal layout from: %s",
        config_path.c_str());
      std::string content((std::istreambuf_iterator<char>(config_file)),
        std::istreambuf_iterator<char>());
      process_terminal_config(content);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to open config file: %s",
        config_path.c_str());
    }
  }

  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = reentrant_group_;
  event_subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "events", 10,
    std::bind(&SdlVizNode::event_callback, this, std::placeholders::_1),
    sub_options);
}

SdlVizNode::~SdlVizNode()
{
  if (renderer_) {
    SDL_DestroyRenderer(renderer_);
  }
  if (window_) {
    SDL_DestroyWindow(window_);
  }
  if (surface_) {
    SDL_FreeSurface(surface_);
  }
  if (font_) {
    TTF_CloseFont(font_);
  }
  TTF_Quit();
  SDL_Quit();
}

void SdlVizNode::run()
{
  double fps = this->get_parameter("fps").as_double();
  auto frame_duration =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / fps));

  while (rclcpp::ok()) {
    auto frame_start = std::chrono::steady_clock::now();
    SDL_Event e;
    while (SDL_PollEvent(&e) != 0) {
      if (e.type == SDL_QUIT) {
        rclcpp::shutdown();
        return;
      }
    }

    render_loop();

    auto frame_end = std::chrono::steady_clock::now();
    auto elapsed = frame_end - frame_start;
    if (elapsed < frame_duration) {
      std::this_thread::sleep_for(frame_duration - elapsed);
    }
  }
}

void SdlVizNode::event_callback(const std_msgs::msg::String::SharedPtr msg)
{
  process_terminal_config(msg->data);
}

void SdlVizNode::process_terminal_config(const std::string & json_data)
{
  using json = nlohmann::json;
  try {
    auto config_array = json::parse(json_data);
    if (!config_array.is_array()) {
      return;
    }

    for (const auto & config : config_array) {
      std::string type = config.value("type", "String");

      if (type == "String") {
        if (!config.contains("topic") || !config.contains("area")) {
          continue;
        }
        std::string topic = config["topic"];
        auto area_json = config["area"];
        SDL_Rect area = {area_json[0], area_json[1], area_json[2],
          area_json[3]};

        double lifetime_sec = config.value("expire", 0.0);
        size_t line_limit = config.value("line_limit", 10);
        size_t wrap_width = config.value("wrap_width", 50);
        bool clear_on_new = config.value("clear_on_new", false);
        bool append_newline = config.value("append_newline", false);

        std::string align_str = config.value("align", "left");
        yTerminalAlign align = ALIGN_LEFT;
        if (align_str == "right") {
          align = ALIGN_RIGHT;
        } else if (align_str == "center") {
          align = ALIGN_CENTER;
        }

        auto text_color_json =
          config.value("text_color", json::array({200, 200, 200, 255}));
        SDL_Color text_color = {
          (Uint8)text_color_json[0], (Uint8)text_color_json[1],
          (Uint8)text_color_json[2], (Uint8)text_color_json[3]};

        auto bg_color_json =
          config.value("bg_color", json::array({30, 30, 30, 180}));
        SDL_Color bg_color = {(Uint8)bg_color_json[0], (Uint8)bg_color_json[1],
          (Uint8)bg_color_json[2], (Uint8)bg_color_json[3]};

        auto dt = std::make_unique<DynamicTerminal>();
        dt->terminal = std::make_unique<yTerminal>(
          font_, line_limit, wrap_width, area, text_color, bg_color, align,
          clear_on_new, append_newline);
        dt->creation_time = this->now();
        dt->lifetime = rclcpp::Duration::from_seconds(lifetime_sec);

        dt->subscriber = this->create_subscription<std_msgs::msg::String>(
          topic, 10,
          [this, topic](const std_msgs::msg::String::SharedPtr sub_msg) {
            std::lock_guard<std::mutex> lock(terminals_mutex_);
            if (dynamic_terminals_.count(topic)) {
              dynamic_terminals_[topic]->terminal->append(sub_msg->data);
            }
          });

        std::lock_guard<std::mutex> lock(terminals_mutex_);
        dynamic_terminals_[topic] = std::move(dt);

      } else if (type == "VideoStream") {
        if (!config.contains("topic") || !config.contains("area")) {
          continue;
        }
        std::string pipe_path = config["topic"];
        auto area_json = config["area"];
        auto vs = std::make_unique<DynamicVideoStream>();
        vs->area = {area_json[0], area_json[1], area_json[2], area_json[3]};
        vs->source_width = config.value("source_width", 640);
        vs->source_height = config.value("source_height", 480);

        vs->texture = SDL_CreateTexture(
          renderer_, SDL_PIXELFORMAT_BGRA32,
          SDL_TEXTUREACCESS_STREAMING,
          vs->source_width, vs->source_height);
        vs->reader = std::make_unique<FifoReader>(
          pipe_path, vs->source_width,
          vs->source_height);
        vs->reader->start();

        std::lock_guard<std::mutex> lock(video_streams_mutex_);
        dynamic_video_streams_[pipe_path] = std::move(vs);

      } else if (type == "MarkerLayer") {
        if (!config.contains("topic") || !config.contains("area")) {
          continue;
        }
        std::string topic = config["topic"];
        auto area_json = config["area"];
        auto ml = std::make_unique<DynamicMarkerLayer>();
        ml->area = {area_json[0], area_json[1], area_json[2], area_json[3]};
        ml->scale = config.value("scale", 1000.0);
        ml->offset_x = config.value("offset_x", 0.0);
        ml->offset_y = config.value("offset_y", 0.0);

        if (config.contains("exclude_ns")) {
          std::string ns_str = config["exclude_ns"];
          std::stringstream ss(ns_str);
          std::string ns;
          while (std::getline(ss, ns, ',')) {
            if (!ns.empty()) {
              ml->excluded_ns.push_back(ns);
            }
          }
        }

        ml->subscriber =
          this->create_subscription<visualization_msgs::msg::MarkerArray>(
          topic, 10,
          [this, topic](
            const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(marker_layers_mutex_);
            if (dynamic_marker_layers_.count(topic)) {
              auto & layer = dynamic_marker_layers_[topic];
              auto sorted_msg =
              std::make_shared<visualization_msgs::msg::MarkerArray>(
                *msg);
              std::sort(
                sorted_msg->markers.begin(),
                sorted_msg->markers.end(),
                [](const visualization_msgs::msg::Marker & a,
                const visualization_msgs::msg::Marker & b) {
                  return a.pose.position.x > b.pose.position.x;
                });
              std::lock_guard<std::mutex> data_lock(layer->data_mutex);
              layer->last_markers = sorted_msg;
            }
          });

        std::lock_guard<std::mutex> lock(marker_layers_mutex_);
        dynamic_marker_layers_[topic] = std::move(ml);
      }
    }
  } catch (...) {
  }
}

void SdlVizNode::render_loop()
{
  SDL_SetRenderDrawColor(renderer_, 0x1E, 0x1E, 0x1E, 0xFF);
  SDL_RenderClear(renderer_);

  {
    std::lock_guard<std::mutex> lock(video_streams_mutex_);
    for (auto const &[pipe_path, vs] : dynamic_video_streams_) {
      if (vs->reader->get_latest_frame(video_frame_buffer_)) {
        if (vs->texture) {
          SDL_UpdateTexture(
            vs->texture, NULL, video_frame_buffer_.data(),
            vs->source_width * 4);
        }
      }
      if (vs->texture) {
        SDL_RenderCopy(renderer_, vs->texture, NULL, &vs->area);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(marker_layers_mutex_);
    for (auto const &[topic, ml] : dynamic_marker_layers_) {
      visualization_msgs::msg::MarkerArray::SharedPtr markers_to_draw;
      {
        std::lock_guard<std::mutex> data_lock(ml->data_mutex);
        markers_to_draw = ml->last_markers;
      }

      if (markers_to_draw && !markers_to_draw->markers.empty()) {
        for (const auto & marker : markers_to_draw->markers) {
          if (!ml->excluded_ns.empty() &&
            std::find(
              ml->excluded_ns.begin(), ml->excluded_ns.end(),
              marker.ns) != ml->excluded_ns.end())
          {
            continue;
          }

          SDL_SetRenderDrawColor(
            renderer_, marker.color.r * 255,
            marker.color.g * 255, marker.color.b * 255,
            255);

          switch (marker.type) {
            case visualization_msgs::msg::Marker::LINE_STRIP: {
                if (marker.points.size() < 2) {
                  break;
                }
                for (size_t i = 0; i < marker.points.size() - 1; ++i) {
                  const auto & p1 = marker.points[i];
                  const auto & p2 = marker.points[i + 1];

                  int x1 = ml->area.x + ml->area.w / 2 +
                    (marker.pose.position.y + p1.y) * ml->scale +
                    ml->offset_x;
                  int y1 = ml->area.y + ml->area.h / 2 -
                    (marker.pose.position.z + p1.z) * ml->scale +
                    ml->offset_y;
                  int x2 = ml->area.x + ml->area.w / 2 +
                    (marker.pose.position.y + p2.y) * ml->scale +
                    ml->offset_x;
                  int y2 = ml->area.y + ml->area.h / 2 -
                    (marker.pose.position.z + p2.z) * ml->scale +
                    ml->offset_y;

                  SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
                }
              } break;

            case visualization_msgs::msg::Marker::SPHERE:
            case visualization_msgs::msg::Marker::CYLINDER: {
                int cx = ml->area.x + ml->area.w / 2 +
                  marker.pose.position.y * ml->scale + ml->offset_x;
                int cy = ml->area.y + ml->area.h / 2 -
                  marker.pose.position.z * ml->scale + ml->offset_y;
                int radius = (marker.scale.y * ml->scale) / 2;

                for (int w = 0; w < radius * 2; w++) {
                  for (int h = 0; h < radius * 2; h++) {
                    int dx = radius - w;
                    int dy = radius - h;
                    if ((dx * dx + dy * dy) <= (radius * radius)) {
                      SDL_RenderDrawPoint(renderer_, cx + dx, cy + dy);
                    }
                  }
                }
              } break;
          }
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(terminals_mutex_);
    auto now = this->now();
    std::vector<std::string> expired_terminals;

    for (auto const &[topic, dt] : dynamic_terminals_) {
      if (dt->lifetime.seconds() > 0 &&
        (dt->creation_time + dt->lifetime) < now)
      {
        expired_terminals.push_back(topic);
      } else {
        dt->terminal->draw(renderer_);
      }
    }

    for (const auto & topic : expired_terminals) {
      dynamic_terminals_.erase(topic);
    }
  }

  SDL_RenderPresent(renderer_);

  if (stream_output_) {
    int pitch = 0;
    void * pixels = nullptr;
    SDL_Surface * render_target = surface_;

    if (render_target) {
      SDL_LockSurface(render_target);
      pixels = render_target->pixels;
      pitch = render_target->pitch;
      if (pixels) {
        streamer_->write_frame(pixels, screen_height_ * pitch);
      }
      SDL_UnlockSurface(render_target);
    }
  }
}
