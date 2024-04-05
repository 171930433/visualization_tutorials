// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// BEGIN_TUTORIAL

// The main() for this "myviz" example is very simple. It just
// initializes rclcpp and creates a QApplication, a node, and a top-level
// widget (of type "MyViz"). It also shows the widget and runs the Qt event loop.

#include <QApplication>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>
#include <rviz_common/logging.hpp>

#include "myviz.hpp"


#include "visualizer_app2.hpp"
// int main2(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);

//   QApplication app(argc, argv);

//   auto ros_node_abs =
//     std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");

//   auto myviz = std::make_shared<MyViz>(&app, ros_node_abs);
//   myviz->show();

//   while (rclcpp::ok()) {
//     app.processEvents();
//   }

//   return 0;
// }


int main(int argc, char ** argv)
{
  // remove ROS arguments before passing to QApplication
  std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> non_ros_args_c_strings;
  for (auto & arg : non_ros_args) {
    non_ros_args_c_strings.push_back(&arg.front());
  }
  int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());

  QApplication qapp(non_ros_argc, non_ros_args_c_strings.data());

  // TODO(wjwwood): use node's logger here in stead
  auto logger = rclcpp::get_logger("rviz2");
  // install logging handlers to route logging through ROS's logging system
  rviz_common::set_logging_handlers(
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_DEBUG(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_INFO(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_WARN(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_ERROR(logger, "%s", msg.c_str());
    }
  );

  rviz_common::VisualizerApp2 vapp(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  vapp.setApp(&qapp);
  if (vapp.init(argc, argv)) {
    return qapp.exec();
  } else {
    return 1;
  }
}