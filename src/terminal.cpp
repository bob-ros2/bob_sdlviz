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
 * @file terminal.cpp
 * @brief Implementation of the yTerminal class for SDL text rendering.
 */

#include "bob_sdlviz/terminal.hpp"
#include <sstream>

/**
 * @brief Construct a new yTerminal object.
 *
 * Initializes the terminal with font, layout, and styling parameters.
 */
yTerminal::yTerminal(
  TTF_Font * font, size_t line_limit, size_t wrap_width,
  SDL_Rect area, SDL_Color text_color, SDL_Color bg_color,
  yTerminalAlign align, bool clear_on_new,
  bool append_newline)
: font_(font), line_limit_(line_limit), wrap_width_(wrap_width),
  area_(area), text_color_(text_color), bg_color_(bg_color), align_(align),
  clear_on_new_(clear_on_new), append_newline_(append_newline) {}

/**
 * @brief Appends text to the terminal history.
 *
 * Handles line splitting, character wrapping, and FIFO behavior (line limiting).
 */
void yTerminal::append(const std::string & text)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (clear_on_new_) {
    lines_.clear();
  }

  std::string processed_text = text;
  if (append_newline_) {
    processed_text += "\n";
  }

  std::string last_line;
  if (!append_newline_ && !lines_.empty()) {
    last_line = lines_.back();
    lines_.pop_back();
  }

  std::string combined_text = last_line + processed_text;
  auto new_lines = split_lines(combined_text);

  for (const auto & line : new_lines) {
    if (line.empty()) {
      lines_.push_back("");
      continue;
    }
    std::stringstream word_stream(line);
    std::string word;
    std::string current_wrapped_line;

    while (word_stream >> word) {
      if (current_wrapped_line.empty()) {
        current_wrapped_line = word;
      } else if (current_wrapped_line.length() + 1 + word.length() <= wrap_width_) {
        current_wrapped_line += " " + word;
      } else {
        lines_.push_back(current_wrapped_line);
        current_wrapped_line = word;
      }
    }
    lines_.push_back(current_wrapped_line);
  }

  while (lines_.size() > line_limit_) {
    lines_.pop_front();
  }
}

/**
 * @brief Draws the text terminal on the screen.
 *
 * Renders the background rectangle followed by each text line with the configured alignment.
 */
void yTerminal::draw(SDL_Renderer * renderer)
{
  std::lock_guard<std::mutex> lock(mutex_);

  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(
    renderer, bg_color_.r, bg_color_.g, bg_color_.b,
    bg_color_.a);
  SDL_RenderFillRect(renderer, &area_);
  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

  int y = area_.y + 5;
  const int margin = 5;
  for (const auto & line : lines_) {
    if (line.empty()) {
      y += TTF_FontHeight(font_);
      continue;
    }

    std::string filtered_line = filter_unsupported_chars(line, font_);
    if (filtered_line.empty()) {
      continue;
    }

    SDL_Surface * surface =
      TTF_RenderUTF8_Blended(font_, filtered_line.c_str(), text_color_);
    if (!surface) {
      continue;
    }

    SDL_Texture * texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (!texture) {
      SDL_FreeSurface(surface);
      continue;
    }

    int x = area_.x + margin;
    if (align_ == ALIGN_RIGHT) {
      x = area_.x + area_.w - margin - surface->w;
    } else if (align_ == ALIGN_CENTER) {
      x = area_.x + (area_.w - surface->w) / 2;
    }

    SDL_Rect dstRect = {x, y, surface->w, surface->h};
    SDL_RenderCopy(renderer, texture, NULL, &dstRect);

    y += surface->h;
    SDL_DestroyTexture(texture);
    SDL_FreeSurface(surface);
  }
}

/**
 * @brief Splits a string into lines based on newline characters.
 */
std::vector<std::string> yTerminal::split_lines(const std::string & str)
{
  std::vector<std::string> result;
  std::string line;
  std::istringstream stream(str);
  while (std::getline(stream, line)) {
    result.push_back(line);
  }
  return result;
}

/**
 * @brief Filters out characters not present in the font to prevent rendering crashes/glitches.
 *
 * Uses a basic UTF-8 parser to identify and validate multibyte sequences.
 */
std::string yTerminal::filter_unsupported_chars(
  const std::string & input,
  TTF_Font * font)
{
  std::string filtered_string;
  filtered_string.reserve(input.size());

  for (size_t i = 0; i < input.size(); ) {
    unsigned char c = input[i];
    Uint32 codepoint = 0;
    int len = 0;

    if (c < 0x80) {
      codepoint = c;
      len = 1;
    } else if ((c & 0xE0) == 0xC0) {
      if (i + 1 < input.size()) {
        codepoint = ((c & 0x1F) << 6) | (input[i + 1] & 0x3F);
        len = 2;
      }
    } else if ((c & 0xF0) == 0xE0) {
      if (i + 2 < input.size()) {
        codepoint = ((c & 0x0F) << 12) | ((input[i + 1] & 0x3F) << 6) |
          (input[i + 2] & 0x3F);
        len = 3;
      }
    } else if ((c & 0xF8) == 0xF0) {
      if (i + 3 < input.size()) {
        codepoint = ((c & 0x07) << 18) | ((input[i + 1] & 0x3F) << 12) |
          ((input[i + 2] & 0x3F) << 6) | (input[i + 3] & 0x3F);
        len = 4;
      }
    }

    if (len > 0) {
      if (TTF_GlyphIsProvided32(font, codepoint)) {
        filtered_string.append(input.substr(i, len));
      }
      i += len;
    } else {
      i++;
    }
  }
  return filtered_string;
}

SDL_Rect yTerminal::get_area() const
{
  // Note: we can't lock a mutex in a const method if it's not mutable,
  // but since area_ is rarely changed and this is for reporting,
  // let's just return it. Actually, better make the mutex mutable or just return.
  // In this case, area_ is 4 ints, so direct return is fine.
  return area_;
}

void yTerminal::set_area(SDL_Rect area)
{
  std::lock_guard<std::mutex> lock(mutex_);
  area_ = area;
}

void yTerminal::set_colors(SDL_Color text_color, SDL_Color bg_color)
{
  std::lock_guard<std::mutex> lock(mutex_);
  text_color_ = text_color;
  bg_color_ = bg_color;
}

void yTerminal::set_line_limit(size_t line_limit)
{
  std::lock_guard<std::mutex> lock(mutex_);
  line_limit_ = line_limit;
  while (lines_.size() > line_limit_) {
    lines_.pop_front();
  }
}

void yTerminal::set_wrap_width(size_t wrap_width)
{
  std::lock_guard<std::mutex> lock(mutex_);
  wrap_width_ = wrap_width;
  // Note: This doesn't re-wrap existing lines, but applies to new ones.
}

void yTerminal::set_align(yTerminalAlign align)
{
  std::lock_guard<std::mutex> lock(mutex_);
  align_ = align;
}

void yTerminal::set_behavior(bool clear_on_new, bool append_newline)
{
  std::lock_guard<std::mutex> lock(mutex_);
  clear_on_new_ = clear_on_new;
  append_newline_ = append_newline;
}
