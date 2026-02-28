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
 * @file terminal.hpp
 * @brief Dynamic text terminal rendering for bob_sdlviz.
 *
 * Defines the yTerminal class which handles multi-line text rendering,
 * word wrapping, and alignment within an SDL-based UI.
 */

#ifndef BOB_SDLVIZ__TERMINAL_HPP_
#define BOB_SDLVIZ__TERMINAL_HPP_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

/**
 * @enum yTerminalAlign
 * @brief Horizontal text alignment options for the terminal.
 */
enum yTerminalAlign { ALIGN_LEFT, ALIGN_RIGHT, ALIGN_CENTER };

/**
 * @class yTerminal
 * @brief A class for rendering a scrolling/dynamic text terminal using SDL2.
 *
 * Supports line limits, word wrapping, background colors, and various alignments.
 */
class yTerminal
{
public:
  /**
   * @brief Construct a new yTerminal object.
   * @param font The TTF font to use for rendering.
   * @param line_limit Maximum number of lines to display.
   * @param wrap_width Maximum width of a line before wrapping (in characters).
   * @param area The screen destination rectangle for the terminal.
   * @param text_color The color of the rendered text.
   * @param bg_color The background color of the terminal area.
   * @param align Text alignment (Left, Right, Center).
   * @param clear_on_new Whether to clear the terminal on every new append.
   * @param append_newline Whether to automatically append a newline to inputs.
   */
  yTerminal(
    TTF_Font * font, size_t line_limit, size_t wrap_width, SDL_Rect area,
    SDL_Color text_color, SDL_Color bg_color, yTerminalAlign align,
    bool clear_on_new, bool append_newline);

  /**
   * @brief Appends new text to the terminal. Performs wrapping and line limiting.
   * @param text The text string to append.
   */
  void append(const std::string & text);

  /**
   * @brief Renders the terminal onto the given SDL renderer.
   * @param renderer The SDL renderer to draw on.
   */
  void draw(SDL_Renderer * renderer);

  /**
   * @brief Returns the current bounding area of the terminal.
   * @return SDL_Rect
   */
  SDL_Rect get_area() const;

  // Setters for dynamic updates
  void set_area(SDL_Rect area);
  void set_colors(SDL_Color text_color, SDL_Color bg_color);
  void set_line_limit(size_t line_limit);
  void set_wrap_width(size_t wrap_width);
  void set_align(yTerminalAlign align);
  void set_behavior(bool clear_on_new, bool append_newline);

private:
  /**
   * @brief Helper to split a string into multiple lines based on wrap_width.
   * @param str The input string.
   * @return std::vector<std::string> List of wrapped lines.
   */
  std::vector<std::string> split_lines(const std::string & str);

  /**
   * @brief Filters out characters not supported by the current font.
   * @param input The raw input string.
   * @param font The font to check against.
   * @return std::string The filtered string.
   */
  std::string filter_unsupported_chars(
    const std::string & input,
    TTF_Font * font);

  TTF_Font * font_;         ///< Pointer to the TTF font surface.
  size_t line_limit_;       ///< Maximum lines kept in history.
  size_t wrap_width_;       ///< Width for automatic word wrapping.
  SDL_Rect area_;           ///< UI bounding box.
  SDL_Color text_color_;    ///< RGBA text color.
  SDL_Color bg_color_;      ///< RGBA background color.
  yTerminalAlign align_;    ///< Current alignment.
  bool clear_on_new_;       ///< Flag for "flash" terminal mode.
  bool append_newline_;     ///< Flag to treat every append as a paragraph.
  std::deque<std::string> lines_;  ///< Queue of active lines.
  std::mutex mutex_;        ///< Mutex for thread-safe appends.
};

#endif  // BOB_SDLVIZ__TERMINAL_HPP_
