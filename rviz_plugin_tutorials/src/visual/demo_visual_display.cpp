#include "visual/demo_visual_display.hpp"

#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreRenderWindow.h>

DemoVisualDisplay::DemoVisualDisplay() : rviz::Display() {}

void DemoVisualDisplay::onInitialize() {
  shape_.initialize(context_, this);
}

void DemoVisualDisplay::update(float wall_dt, float ros_dt) {
  shape_.update();
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(DemoVisualDisplay, rviz::Display)