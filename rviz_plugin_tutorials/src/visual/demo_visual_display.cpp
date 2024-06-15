#include "visual/demo_visual_display.hpp"

#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreRenderWindow.h>


using namespace rviz_visual_tools;

void testSize(double &x_location, scales scale, RvizVisualTools *visual_tools_) {
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
  pose2 = pose1;
  pose2.translation().x() += step / 2.0;
  points1.emplace_back(pose1.translation());
  points2.emplace_back(pose2.translation());
  pose1.translation().x() += step / 2.0;

  pose2 = pose1;
  pose2.translation().x() += step / 2.0;
  // points1.push_back(pose1.translation());
  // points2.push_back(pose2.translation());
  colors.clear(); // temp
  colors.push_back(ORANGE);
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

  // Display test
  // visual_tools_->trigger();

  // Set x location for next visualization function
  x_location += 0.5;
}

DemoVisualDisplay::DemoVisualDisplay() : rviz::Display() {
  rvt_.reset(new MyRvizVisualTools("map", "/rviz_visual_tools"));
  rvt_->enableBatchPublishing();
}

void DemoVisualDisplay::onInitialize() {

  // shape_.initialize(context_, this);

  // bline_.initialize(context_, this);

  // pc_.initialize(this, context_);

  // 发送
  rvt_->initialize(this, context_);

  double x = 0;
  testSize(x, scales::LARGE, rvt_.get());
}

void DemoVisualDisplay::update(float wall_dt, float ros_dt) {
  // shape_.update();
  // bline_.update();

  rvt_->update();
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(DemoVisualDisplay, rviz::Display)