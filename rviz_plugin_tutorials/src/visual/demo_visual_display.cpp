#include "visual/demo_visual_display.hpp"

#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreRenderWindow.h>

DemoVisualDisplay::DemoVisualDisplay() : rviz::Display() {}

void DemoVisualDisplay::onInitialize() {
  shape_.initialize(context_, this);

  bline_.initialize(context_, this);

  pc_.initialize(this, context_);
}

void DemoVisualDisplay::update(float wall_dt, float ros_dt) {
  shape_.update();
  bline_.update();

  pc_.update(wall_dt, ros_dt);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(DemoVisualDisplay, rviz::Display)