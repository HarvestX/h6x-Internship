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

#include "diff_drive_controller_plugin.hpp"

namespace gazebo_plugins
{
DiffDriveControllerPlugin::DiffDriveControllerPlugin() {}
DiffDriveControllerPlugin::~DiffDriveControllerPlugin() {}

void DiffDriveControllerPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(this->ros_node_->get_logger(), "Loaded GazeboRosDiffDriveControllerPlugin");

  this->speed_limit_ = 1.0;
  this->model_ = _model;
  this->update_conn_ =
    gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(
      &DiffDriveControllerPlugin::OnUpdate,
      this));

  this->cmd_vel_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,

    std::bind(&DiffDriveControllerPlugin::twistCallback, this, std::placeholders::_1));

  this->sub_speedlimit_ = this->ros_node_->create_subscription<std_msgs::msg::Float32>(
    "/speed_limit", 10,
    std::bind(&DiffDriveControllerPlugin::sub_speedlimit_callback, this, std::placeholders::_1));
}

void DiffDriveControllerPlugin::OnUpdate()
{
  gazebo::physics::WorldPtr world_ = this->model_->GetWorld();
  gazebo::physics::LinkPtr link_ = this->model_->GetLink("body_link");
  auto left_wheel_joint = this->model_->GetJoint("left_wheel_joint");
  auto right_wheel_joint = this->model_->GetJoint("right_wheel_joint");

  gazebo::common::Time current_time = world_->SimTime();
  if ((current_time - this->last_time_).Double() > (1.0 / 2.0)) {
    this->last_time_ = current_time;
    ignition::math::Pose3d pose = link_->WorldPose();
    printf("pos: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
    printf("rot: %f %f %f\n", pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());
  }

  float vel_x = this->last_cmd_vel_.linear.x;
  float rot_z = this->last_cmd_vel_.angular.z;

  // speed limit
  if (vel_x > this->speed_limit_) {
    vel_x = this->speed_limit_;
  }
  else if (vel_x < -this->speed_limit_) {
    vel_x = -this->speed_limit_;
  }
  if (rot_z > this->speed_limit_) {
    rot_z = this->speed_limit_;
  }
  else if (rot_z < -this->speed_limit_) {
    rot_z = -this->speed_limit_;
  }

  float wheel_radius = 0.05;
  float wheel_distance = 0.22;

  left_wheel_joint->SetVelocity(0, vel_x / wheel_radius - rot_z * wheel_distance / wheel_radius);
  right_wheel_joint->SetVelocity(0, vel_x / wheel_radius + rot_z * wheel_distance / wheel_radius);
}

void DiffDriveControllerPlugin::Reset()
{
}

void DiffDriveControllerPlugin::twistCallback(const geometry_msgs::msg::Twist & msg)
{
  this->last_cmd_vel_ = msg;
}

void DiffDriveControllerPlugin::sub_speedlimit_callback(const std_msgs::msg::Float32 & msg)
{
  printf("set speed limit: %f\n", msg.data);
  this->speed_limit_ = msg.data;
}

GZ_REGISTER_MODEL_PLUGIN(DiffDriveControllerPlugin)
} // end namespace gazebo_plugins
