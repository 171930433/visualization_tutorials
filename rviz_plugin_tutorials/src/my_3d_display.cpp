#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreViewport.h>

#include <QDebug>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/render_window.hpp>

using namespace rviz_common;

My3dDisplay::My3dDisplay() : rviz_common::Display() {}

void My3dDisplay::setupRenderPanel() {
  static int count = 0;
  render_panel_ = std::make_unique<rviz_common::RenderPanel>();

  render_panel_->resize(640, 480);
  render_panel_->initialize(context_, true);
  setAssociatedWidget(render_panel_.get());

  render_panel_->getRenderWindow()->setObjectName("My3dDisplayRenderWindow" + QString::number(count++));
  // render_panel_->getRenderWindow()->initialize();
}

void My3dDisplay::initialize(rviz_common::DisplayContext *context) {
  Display::initialize(context);
  setupRenderPanel();

  view_manager_ = std::make_shared<rviz_common::ViewManager>(context_);
  view_manager_->setRenderPanel(render_panel_.get());
  view_manager_->initialize();

  view_manager_->setCurrentViewControllerType("rviz_default_plugins/TopDownOrtho");

  inited_ = true;
}

void My3dDisplay::update(float wall_dt, float ros_dt) {
  updateCamera();
  if (inited_ && view_manager_) {
    // !不太确定为什么getViewport的初始化像是异步的
    auto vp = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_panel_->getRenderWindow())->getViewport();
    if (vp) { view_manager_->update(wall_dt, ros_dt); }
  }

  // view_manager_->getCurrent()->update(wall_dt,ros_dt);
}

bool My3dDisplay::updateCamera() {
  // auto render_window = render_panel_->getRenderWindow();
  // auto proj_matrix = view_manager_->getCurrent()->getCamera()->getProjectionMatrix();
  // std::cout << " proj_matrix = " << proj_matrix << "\n";
  // rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_window)->setCustomProjectionMatrix(true, proj_matrix);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(My3dDisplay, rviz_common::Display)