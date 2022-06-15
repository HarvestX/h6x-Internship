#pragma once

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace goal_plugin
{

class GazeboRosTargetPointPrivate;

class GazeboRosTargetPoint : public gazebo::SensorPlugin
{
public:
  GazeboRosTargetPoint();

  virtual ~GazeboRosTargetPoint();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  std::unique_ptr<GazeboRosTargetPointPrivate> impl_;
};

}
