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

#include <gazebo/transport/transport.hh>
#include <string>
#include <algorithm>
#include <limits>
#include <memory>
#include <boost/make_shared.hpp>
#include <boost/variant.hpp>
#include <gazebo_ros/conversions/sensor_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include "gazebo_ros_target_point.hpp"

#include <std_msgs/msg/bool.hpp>

namespace goal_plugin
{

class GazeboRosTargetPointPrivate
{
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  using GoaledPub = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
  boost::variant<GoaledPub> pub_;

  std::string frame_name_;

  /// Subscribe to gazebo's laser scan,
  // calling the appropriate callback based on output type
  void SubscribeGazeboLaserScan();
  void PublishGoaled(const ConstLaserScanStampedPtr & _msg);

  std::string sensor_topic_;

  // double min_intensity_{0.0};

  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::SubscriberPtr laser_scan_sub_;
};

GazeboRosTargetPoint::GazeboRosTargetPoint()
: impl_(std::make_unique<GazeboRosTargetPointPrivate>())
{
}

GazeboRosTargetPoint::~GazeboRosTargetPoint()
{
  // Must release subscriber and then call
  // fini on node to remove it from topic manager.
  impl_->laser_scan_sub_.reset();
  if (impl_->gazebo_node_) {
    impl_->gazebo_node_->Fini();
  }
  impl_->gazebo_node_.reset();
}

void GazeboRosTargetPoint::Load(
  gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Create ros_node configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Get QoS profile for the publisher
  rclcpp::QoS pub_qos =
    qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Get output type from sdf if provided
  impl_->pub_ =
    impl_->ros_node_->create_publisher<std_msgs::msg::Bool>("~/out", pub_qos);
  // Create gazebo transport node and subscribe to sensor's laser scan
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_sensor->WorldName());

  impl_->sensor_topic_ = _sensor->Topic();
  impl_->SubscribeGazeboLaserScan();
}

void GazeboRosTargetPointPrivate::SubscribeGazeboLaserScan()
{
  laser_scan_sub_ = gazebo_node_->Subscribe(
    sensor_topic_, &GazeboRosTargetPointPrivate::PublishGoaled, this);
}

void GazeboRosTargetPointPrivate::PublishGoaled(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to range
  auto range_msg = gazebo_ros::Convert<sensor_msgs::msg::Range>(*_msg);
  bool is_goal = false;
  float touch_distance = 1.0;
  std_msgs::msg::Bool goaled_msg;

  if (range_msg.range < touch_distance) {
    is_goal = true;
  }

  goaled_msg.data = is_goal;
  boost::get<GoaledPub>(pub_)->publish(goaled_msg);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosTargetPoint)

}  // namespace goal_plugin
