#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreRenderWindow.h>

#include <rviz/default_plugin/view_controllers/fixed_orientation_ortho_view_controller.h>
#include <rviz/viewport_mouse_event.h>

namespace rviz {

class MyFixedOrientationOrthoViewController : public FixedOrientationOrthoViewController {
  using FixedOrientationOrthoViewController::FixedOrientationOrthoViewController;

  void handleMouseEvent(ViewportMouseEvent &event) override {
    event.modifiers.setFlag(Qt::ShiftModifier); // 强制shift锁定
    FixedOrientationOrthoViewController::handleMouseEvent(event);
  }
};

} // namespace rviz

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

  view_manager_->setCurrentViewControllerType("rviz/MyTopDownOrtho");

  inited_ = true;
}

void My3dDisplay::update(float wall_dt, float ros_dt) { view_manager_->update(wall_dt, ros_dt); }

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(My3dDisplay, rviz::Display)

PLUGINLIB_EXPORT_CLASS(rviz::MyFixedOrientationOrthoViewController, rviz::ViewController)