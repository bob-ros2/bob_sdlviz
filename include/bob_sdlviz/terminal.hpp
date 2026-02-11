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

#ifndef BOB_SDLVIZ__TERMINAL_HPP_
#define BOB_SDLVIZ__TERMINAL_HPP_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

enum yTerminalAlign { ALIGN_LEFT, ALIGN_RIGHT, ALIGN_CENTER };

class yTerminal
{
public:
  yTerminal(
    TTF_Font * font, size_t line_limit, size_t wrap_width, SDL_Rect area,
    SDL_Color text_color, SDL_Color bg_color, yTerminalAlign align,
    bool clear_on_new, bool append_newline);

  void append(const std::string & text);
  void draw(SDL_Renderer * renderer);

private:
  std::vector<std::string> split_lines(const std::string & str);
  std::string filter_unsupported_chars(
    const std::string & input,
    TTF_Font * font);

  TTF_Font * font_;
  size_t line_limit_;
  size_t wrap_width_;
  SDL_Rect area_;
  SDL_Color text_color_;
  SDL_Color bg_color_;
  yTerminalAlign align_;
  bool clear_on_new_;
  bool append_newline_;
  std::deque<std::string> lines_;
  std::mutex mutex_;
};

#endif  // BOB_SDLVIZ__TERMINAL_HPP_
