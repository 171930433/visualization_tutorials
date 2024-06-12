
#pragma once

#include <rviz/default_plugin/point_cloud_common.h>
#include <Eigen/Dense>

#include <rviz/properties/bool_property.h>


class MyPointCloud : public QObject {
public:
public:
  void initialize(rviz::Display *display,rviz::DisplayContext *context);

  void update(float wall_dt, float ros_dt);

private:
  rviz::DisplayContext *context_ = nullptr;
  rviz::BoolProperty *shape_property_;
  std::shared_ptr<rviz::PointCloudCommon> pc_common_;

  std::vector<Eigen::Vector3f> pts_;


};