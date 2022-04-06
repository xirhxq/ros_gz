// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

#include "bridge.hpp"
#include "bridge_ign_to_ros.hpp"
#include "bridge_ros_to_ign.hpp"

// Direction of bridge.
enum Direction
{
  // Both directions.
  BIDIRECTIONAL = 0,
  // Only from IGN to ROS
  FROM_IGN_TO_ROS = 1,
  // Only from ROS to IGN
  FROM_ROS_TO_IGN = 2,
  // Unspecified, only used for services
  DIR_UNSPECIFIED = 3,
};

//////////////////////////////////////////////////
void usage()
{
  std::cout << "Bridge a collection of ROS2 and Ignition Transport topics and services.\n\n" <<
    "  parameter_bridge [<topic@ROS2_type@Ign_type> ..] " <<
    " [<service@ROS2_srv_type[@Ign_req_type@Ign_rep_type]> ..]\n\n" <<
    "Topics: The first @ symbol delimits the topic name from the message types.\n" <<
    "Following the first @ symbol is the ROS message type.\n" <<
    "The ROS message type is followed by an @, [, or ] symbol where\n" <<
    "    @  == a bidirectional bridge, \n" <<
    "    [  == a bridge from Ignition to ROS,\n" <<
    "    ]  == a bridge from ROS to Ignition.\n" <<
    "Following the direction symbol is the Ignition Transport message " <<
    "type.\n\n" <<
    "Services: The first @ symbol delimits the service name from the types.\n" <<
    "Following the first @ symbol is the ROS service type.\n" <<
    "Optionally, you can include the Ignition request and response type\n" <<
    "separated by the @ symbol.\n" <<
    "It is only supported to expose Ignition servces as ROS services, i.e.\n"
    "the ROS service will forward request to the Ignition service and then forward\n"
    "the response back to the ROS client.\n\n"
    "A bidirectional bridge example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String@ignition.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from Ignition to ROS example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String[ignition.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from ROS to Ignition example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String]ignition.msgs" <<
    ".StringMsg\n" <<
    "A service bridge:\n" <<
    "    parameter_bridge /world/default/control@ros_ign_interfaces/srv/ControlWorld\n" <<
    "Or equivalently:\n" <<
    "    parameter_bridge /world/default/control@ros_ign_interfaces/srv/ControlWorld@"
    "ignition.msgs.WorldControl@ignition.msgs.Boolean\n" << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2) {
    usage();
    return -1;
  }
  // skip the process name in argument procesing
  ++argv;
  --argc;
  auto filteredArgs = rclcpp::init_and_remove_ros_arguments(argc, argv);

  // ROS 2 node
  auto ros_node = std::make_shared<rclcpp::Node>("ros_ign_bridge");

  ros_node->declare_parameter<bool>("lazy", false);

  bool lazy_subscription;
  ros_node->get_parameter("lazy", lazy_subscription);

  // Ignition node
  auto ign_node = std::make_shared<ignition::transport::Node>();

  // Filter arguments (i.e. remove ros args) then parse all the remaining ones
  // TODO(ivanpauno): Improve the parsing code later, it's hard to read ...
  const std::string delim = "@";

  std::list<ros_ign_bridge::BridgePtr> handles;
  std::vector<ros_ign_bridge::BridgeIgnServicesToRosHandles> service_bridge_handles;

  for (auto & arg : filteredArgs) {
    auto delimPos = arg.find(delim);
    if (delimPos == std::string::npos || delimPos == 0) {
      usage();
      return -1;
    }
    std::string topic_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    // Get the direction delimeter, which should be one of:
    //   @ == bidirectional, or
    //   [ == only from IGN to ROS, or
    //   ] == only from ROS to IGN.
    delimPos = arg.find("@");
    Direction direction = BIDIRECTIONAL;
    if (delimPos == std::string::npos || delimPos == 0) {
      delimPos = arg.find("[");
      if (delimPos == std::string::npos || delimPos == 0) {
        delimPos = arg.find("]");
        if (delimPos == 0) {
          usage();
          return -1;
        } else if (delimPos == std::string::npos) {
          direction = DIR_UNSPECIFIED;
        } else {
          direction = FROM_ROS_TO_IGN;
        }
      } else {
        direction = FROM_IGN_TO_ROS;
      }
    }
    std::string ros_type_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());
    if (ros_type_name.find("/srv/") != std::string::npos) {
      std::string ign_req_type_name;
      std::string ign_rep_type_name;
      if (direction != DIR_UNSPECIFIED && direction != BIDIRECTIONAL) {
        usage();
        return -1;
      }
      if (direction == BIDIRECTIONAL) {
        delimPos = arg.find("@");
        if (delimPos == std::string::npos || delimPos == 0) {
          usage();
          return -1;
        }
        ign_req_type_name = arg.substr(0, delimPos);
        arg.erase(0, delimPos + delim.size());
        ign_rep_type_name = std::move(arg);
      }
      try {
        service_bridge_handles.push_back(
          ros_ign_bridge::create_service_bridge(
            ros_node,
            ign_node,
            ros_type_name,
            ign_req_type_name,
            ign_rep_type_name,
            topic_name));
      } catch (std::runtime_error & e) {
        std::cerr << e.what() << std::endl;
      }
      continue;
    }

    delimPos = arg.find(delim);
    if (delimPos != std::string::npos || arg.empty()) {
      usage();
      return -1;
    }
    std::string ign_type_name = arg;

    try {
      if (direction == FROM_IGN_TO_ROS || direction == BIDIRECTIONAL) {
        RCLCPP_INFO(
          ros_node->get_logger(),
          "Creating IGN->ROS Bridge: [%s] (%s -> %s) (Lazy %d): ",
          topic_name.c_str(), ign_type_name.c_str(), ros_type_name.c_str(),
          lazy_subscription);
        handles.push_back(
          std::make_unique<ros_ign_bridge::BridgeIgnToRos>(
            ros_node, ign_node,
            ros_type_name, topic_name,
            ign_type_name, topic_name,
            ros_ign_bridge::Bridge::kDefaultSubscriberQueue,
            ros_ign_bridge::Bridge::kDefaultPublisherQueue,
            lazy_subscription
        ));
      }
      if (direction == FROM_ROS_TO_IGN || direction == BIDIRECTIONAL) {
        RCLCPP_INFO(
          ros_node->get_logger(),
          "Creating ROS->IGN Bridge: [%s] (%s -> %s) (Lazy %d): ",
          topic_name.c_str(), ros_type_name.c_str(), ign_type_name.c_str(),
          lazy_subscription);
        handles.push_back(
          std::make_unique<ros_ign_bridge::BridgeRosToIgn>(
            ros_node, ign_node,
            ros_type_name, topic_name,
            ign_type_name, topic_name,
            ros_ign_bridge::Bridge::kDefaultSubscriberQueue,
            ros_ign_bridge::Bridge::kDefaultPublisherQueue,
            lazy_subscription
        ));
      }
    } catch (std::runtime_error & _e) {
      RCLCPP_WARN(
        ros_node->get_logger(),
        "Failed to create a bridge for topic [%s] with ROS2 type [%s] "
        "and Ignition Transport type[%s]",
        topic_name.c_str(),
        ros_type_name.c_str(),
        ign_type_name.c_str());
    }
  }

  for (auto & bridge : handles) {
    bridge->Start();
  }

  auto timer = ros_node->create_wall_timer(
    std::chrono::milliseconds(1000),
    [&handles]() {
      for (auto & bridge : handles) {
        bridge->Spin();
      }
    });

  // ROS 2 spinner
  rclcpp::spin(ros_node);

  // Wait for ign node shutdown
  ignition::transport::waitForShutdown();

  return 0;
}
