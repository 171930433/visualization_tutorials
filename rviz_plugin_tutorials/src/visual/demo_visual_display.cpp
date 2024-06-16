#include "visual/demo_visual_display.hpp"

#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreRenderWindow.h>

using namespace rviz_visual_tools;

void testSize(double &x_location, scales scale, MyRvizVisualTools *visual_tools_);

void DemoVisualDisplay::setTopic(const QString &topic, const QString &datatype) {
  ROS_INFO_STREAM("DemoVisualDisplay::setTopic called" << topic.toStdString());
}

DemoVisualDisplay::DemoVisualDisplay() : rviz::Display() {
  rvt_.reset(new MyRvizVisualTools("map"));
}

void DemoVisualDisplay::onInitialize() {
  // 发送
  rvt_->initialize(this, context_);

  rvt_->beginInit();

  double x = 0;
  for (int i = 0; i < 10; ++i) {
    testSize(x, scales::LARGE, rvt_.get());
    x += 1;
  }

  rvt_->endInit();
}

void DemoVisualDisplay::update(float wall_dt, float ros_dt) {
  //
  rvt_->update();
}

void testSize(double &x_location, scales scale, MyRvizVisualTools *visual_tools_) {
  using namespace Eigen;
  // Create pose
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();

  // Reusable vector of 2 colors
  std::vector<colors> colors;
  colors.push_back(RED);
  colors.push_back(GREEN);

  // Reusable points vector
  EigenSTL::vector_Vector3d points1;
  EigenSTL::vector_Vector3d points2;

  double step = 0.25; // space between each row

  // Show test label
  pose1.translation().x() = x_location - 0.1;
  visual_tools_->publishText(
      pose1, "Testing consistency of " + visual_tools_->scaleToString(scale) + " marker scale", WHITE, XLARGE, false);

  pose1.translation().x() = x_location;

  // TODO(dave): publishCone() - no scale version available
  // TODO(dave): publishXYPlane() - no scale version available
  // TODO(dave): publishXZPlane() - no scale version available
  // TODO(dave): publishYZPlane() - no scale version available

  // Sphere
  visual_tools_->publishSphere(pose1, BLUE, scale);
  pose1.translation().y() += step;

  // Spheres
  points1.clear();
  points1.emplace_back(pose1.translation());
  pose1.translation().x() += step;
  points1.emplace_back(pose1.translation());
  visual_tools_->publishSpheres(points1, BLUE, scale);
  pose1.translation().x() = x_location; // reset
  pose1.translation().y() += step;

  // Spheres with colors
  points1.clear();
  points1.emplace_back(pose1.translation());
  pose1.translation().x() += step;
  points1.emplace_back(pose1.translation());
  visual_tools_->publishSpheres(points1, colors, scale);
  pose1.translation().x() = x_location; // reset
  pose1.translation().y() += step;

  // YArrow
  visual_tools_->publishYArrow(pose1, BLUE, scale);
  pose1.translation().y() += step;

  // ZArrow
  visual_tools_->publishZArrow(pose1, GREEN, scale);
  pose1.translation().y() += step;

  // XArrow
  visual_tools_->publishXArrow(pose1, RED, scale);
  pose1.translation().y() += step;

  // Arrow (x arrow)
  visual_tools_->publishArrow(pose1, RED, scale);
  pose1.translation().y() += step;

  // Line
  pose2 = pose1;
  pose2.translation().x() += step / 2.0;
  visual_tools_->publishLine(pose1, pose2, PURPLE, scale);
  pose1.translation().y() += step;

  // Lines
  points1.clear();
  points2.clear();
  colors.clear(); // temp

  pose2 = pose1;
  pose2.translation().x() += step / 2.0;
  for (int i = 0; i < 3; ++i) {
    points1.emplace_back(pose1.translation() + Eigen::Vector3d{i * 1.0, 0, 0} * step);
    points2.emplace_back(pose2.translation() + Eigen::Vector3d{i * 1.0, 0, 0} * step);
    colors.push_back((rviz_visual_tools::colors)(i + 1));
  }
  pose1.translation().x() += step / 2.0;

  pose2 = pose1;
  pose2.translation().x() += step / 2.0;

  visual_tools_->publishLines(points1, points2, colors, scale);
  pose1.translation().x() = x_location; // reset
  pose1.translation().y() += step;

  // TODO(dave): publishPath
  // TODO(dave): publishPolygon
  // TODO(dave): publishWireframeCuboid
  // TODO(dave): publishWireframeRectangle

  // Axis Labeled
  visual_tools_->publishAxisLabeled(pose1, "Axis", scale);
  pose1.translation().y() += step;

  // Axis
  visual_tools_->publishAxis(pose1, scale);
  pose1.translation().y() += step;

  // TODO(dave): publishAxis

  // Cylinder
  pose2 = pose1;
  pose2.translation().x() += step / 2.0;
  visual_tools_->publishCylinder(pose1.translation(), pose2.translation(), BLUE, scale);
  pose1.translation().y() += step;

  // TODO(dave): publishMesh

  // TODO(dave): publishGraph

  // Text
  visual_tools_->publishText(pose1, "Text", WHITE, scale, false);
  pose1.translation().y() += step;

  // add 平面
  visual_tools_->publishRect(pose1.translation(), Vector3d::UnitX(), RED, 0.1, 0.1);
  visual_tools_->publishRect(pose1.translation(), Vector3d::UnitY(), GREEN, 0.1, 0.1);
  visual_tools_->publishRect(pose1.translation(), Vector3d::UnitZ(), BLUE, 0.1, 0.1);
  pose1.translation().y() += step;

  // add 圆
  visual_tools_->publishCircle(pose1.translation(), Vector3d::UnitX(), 0.1, RED);
  visual_tools_->publishCircle(pose1.translation(), Vector3d::UnitY(), 0.1, GREEN);
  visual_tools_->publishCircle(pose1.translation(), Vector3d::UnitZ(), 0.1, BLUE);
  pose1.translation().y() += step;

  // add 三角形
  visual_tools_->publishRegularTriangle(pose1.translation(), Vector3d::UnitX(), 0.1, RED);
  visual_tools_->publishRegularTriangle(pose1.translation(), Vector3d::UnitY(), 0.1, GREEN);
  visual_tools_->publishRegularTriangle(pose1.translation(), Vector3d::UnitZ(), 0.1, BLUE);
  pose1.translation().y() += step;

  // 正多边形
  for (int i = 3; i < 10; ++i) {
    visual_tools_->publishRegularPolygon2(pose1.translation(), Vector3d::UnitX(), i, 0.1, RED);
    visual_tools_->publishRegularPolygon2(pose1.translation(), Vector3d::UnitY(), i, 0.1, GREEN);
    visual_tools_->publishRegularPolygon2(pose1.translation(), Vector3d::UnitZ(), i, 0.1, BLUE);
    pose1.translation().y() += step;
  }

  // Set x location for next visualization function
  x_location += 0.5;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(DemoVisualDisplay, rviz::Display)