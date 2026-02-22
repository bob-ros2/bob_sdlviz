cmake_minimum_required(VERSION 3.8)
project(bob_sdlviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# Use pkg-config to find system libraries like SDL2. This is the most reliable method.
find_package(PkgConfig REQUIRED)
find_package(nlohmann_json REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
pkg_check_modules(SDL2_TTF REQUIRED IMPORTED_TARGET SDL2_ttf)

add_executable(sdlviz
  src/sdlviz_node.cpp
  src/sdlviz.cpp
  src/terminal.cpp
)

target_include_directories(sdlviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sdlviz
  rclcpp
  std_msgs
  visualization_msgs
)

# Link against the imported targets created by pkg-config
target_link_libraries(sdlviz
  PkgConfig::SDL2
  PkgConfig::SDL2_TTF
  nlohmann_json::nlohmann_json
)

install(TARGETS
  sdlviz
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/audio_bridge.py
  scripts/feed_audio.sh
  scripts/start_stream.sh
  scripts/write_fifo.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()