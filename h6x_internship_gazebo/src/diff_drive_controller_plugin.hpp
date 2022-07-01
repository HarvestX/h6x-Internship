// Copyright 2022 HarvestX Inc.
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

#pragma once

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sdf/sdf.hh>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class DiffDriveControllerPlugin : public gazebo::ModelPlugin
{
  gazebo::event::ConnectionPtr update_conn_;

public:
  DiffDriveControllerPlugin();
  ~DiffDriveControllerPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf) override;
  void Reset() override;
  void OnUpdate();

  void onTwist(const geometry_msgs::msg::Twist & msg);

  void onSpeedLimit(const std_msgs::msg::Float32 & msg);

private:
  gazebo::physics::ModelPtr model_;
  gazebo::common::Time last_time_;

  float speed_limit_;

  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_limit_;

  geometry_msgs::msg::Twist last_cmd_vel_;
};
}  // end namespace gazebo_plugins
