#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreRenderWindow.h>

#include <rviz/properties/display_group_visibility_property.h>

My3dDisplay::My3dDisplay() : rviz::Display() {

}

My3dDisplay::~My3dDisplay() {
  if (initialized()) { 
    this->takeChild(top_down_view_);
    context_->visibilityBits()->freeBits(vis_bit_); }
}

void My3dDisplay::setupRenderPanel() {
  using namespace rviz;

  static int count = 0;
  render_panel_ = std::make_unique<rviz::RenderPanel>();

  render_panel_->resize(640, 480);
  render_panel_->initialize(context_->getSceneManager(), context_);
  setAssociatedWidget(render_panel_.get());

  // set vis_bit_
  vis_bit_ = context_->visibilityBits()->allocBit();
  render_panel_->getViewport()->setVisibilityMask(vis_bit_);

  visibility_property_ =
      new DisplayGroupVisibilityProperty(vis_bit_,
                                         context_->getRootDisplayGroup(),
                                         this,
                                         "Visibility",
                                         true,
                                         "Changes the visibility of other Displays in the camera view.");

  visibility_property_->setIcon(loadPixmap("package://rviz/icons/visibility.svg", true));

  this->addChild(visibility_property_, 0);
}

void My3dDisplay::onInitialize() {
  setupRenderPanel();

  view_manager_ = std::make_shared<rviz::ViewManager>(context_);
  view_manager_->setRenderPanel(render_panel_.get());

  view_manager_->setCurrentViewControllerType("rviz/MyTopDownOrtho");
  top_down_view_ = qobject_cast<rviz::FixedOrientationOrthoViewController *>(view_manager_->getCurrent());

  this->addChild(top_down_view_, 1);
}

void My3dDisplay::update(float wall_dt, float ros_dt) { view_manager_->update(wall_dt, ros_dt); }

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(My3dDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(rviz::MyFixedOrientationOrthoViewController, rviz::ViewController)