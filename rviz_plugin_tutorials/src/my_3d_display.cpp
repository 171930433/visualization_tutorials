#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>

#include <QDebug>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/render_window.hpp>

using namespace rviz_common;
using namespace rviz_rendering;

void MyShape::initialize(rviz_common::DisplayContext *context, rviz_common::properties::Property *parent) {
  context_ = context;
  shape_property_ = new rviz_common::properties::BoolProperty("MyShape", true, "", parent);
  connect(shape_property_, &properties::Property::changed, this, &MyShape::ColorChanged);

  shape_ = std::make_shared<rviz_rendering::Shape>(Shape::Cone, context->getSceneManager());
  handler_ = rviz_common::interaction::createSelectionHandler<MySelectionHandler>(shape_.get(), context);
  handler_->addTrackedObjects(shape_->getRootNode());

  type_property_ = new properties::EnumProperty("shape_type", "Cone", "", shape_property_);
  type_property_->addOption("Cone", 0);
  type_property_->addOption("Cube", 1);
  type_property_->addOption("Cylinder", 2);
  type_property_->addOption("Sphere", 3);
  connect(type_property_, &properties::Property::changed, this, &MyShape::ShapeChanged);

  pos_property_ = new properties::VectorProperty("pos", Ogre::Vector3::ZERO, "", shape_property_);
  connect(pos_property_, &properties::Property::changed, this, &MyShape::ColorChanged);

  color_property_ = new properties::ColorProperty("color", Qt::black, "", shape_property_);
  connect(color_property_, &properties::Property::changed, this, &MyShape::ColorChanged);
}

void MyShape::update() {
  if (shape_changed_) {
    int type = type_property_->getOptionInt();
    shape_ = std::make_shared<rviz_rendering::Shape>(Shape::Type(type), context_->getSceneManager());
    handler_ = rviz_common::interaction::createSelectionHandler<MySelectionHandler>(shape_.get(), context_);
    handler_->addTrackedObjects(shape_->getRootNode());
    shape_changed_.store(false);
    changed_ = true;
  }
  if (changed_) {
    shape_->getRootNode()->setVisible(shape_property_->getBool());
    shape_->setColor(color_property_->getOgreColor());
    shape_->setPosition(pos_property_->getVector());
    changed_.store(false);
  }
}

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

  shape2_.initialize(context_, this);
  shape3_.initialize(context_, this);
}

void My3dDisplay::update(float wall_dt, float ros_dt) {
  updateCamera();
  if (inited_ && view_manager_) {
    // !不太确定为什么getViewport的初始化像是异步的
    auto vp = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_panel_->getRenderWindow())->getViewport();
    if (vp) { view_manager_->update(wall_dt, ros_dt); }
  }

  // view_manager_->getCurrent()->update(wall_dt,ros_dt);

  shape2_.update();
  shape3_.update();
}

bool My3dDisplay::updateCamera() {
  // auto render_window = render_panel_->getRenderWindow();
  // auto proj_matrix = view_manager_->getCurrent()->getCamera()->getProjectionMatrix();
  // std::cout << " proj_matrix = " << proj_matrix << "\n";
  // rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_window)->setCustomProjectionMatrix(true, proj_matrix);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(My3dDisplay, rviz_common::Display)