#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreRenderWindow.h>

My3dDisplay::My3dDisplay() : rviz::Display() {}

void My3dDisplay::setupRenderPanel() {
  static int count = 0;
  render_panel_ = std::make_unique<rviz::RenderPanel>();

  render_panel_->resize(640, 480);
  render_panel_->initialize(context_->getSceneManager(), context_);
  setAssociatedWidget(render_panel_.get());

}

void My3dDisplay::onInitialize() {
  setupRenderPanel();

  view_manager_ = std::make_shared<rviz::ViewManager>(context_);
  view_manager_->setRenderPanel(render_panel_.get());
  view_manager_->initialize();

  view_manager_->setCurrentViewControllerType("rviz/TopDownOrtho");

  inited_ = true;
}

void My3dDisplay::update(float wall_dt, float ros_dt) {
  view_manager_->update(wall_dt, ros_dt);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(My3dDisplay, rviz::Display)