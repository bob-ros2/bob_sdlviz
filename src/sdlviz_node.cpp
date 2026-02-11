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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SdlVizNode>();

  // Use a background thread for ROS callbacks
  // SDL loop MUST be in the main thread for some platforms,
  // and it's cleaner to keep ROS in the background.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
    4);
  executor.add_node(node);
  std::thread ros_thread([&executor]() {executor.spin();});

  // Run the SDL loop in the main thread
  node->run();

  // If run() returns, it's time to shut down
  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }
  return 0;
}
