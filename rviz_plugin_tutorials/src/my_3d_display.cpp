#include "my_3d_display.hpp"

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>
#include <OgreWireBoundingBox.h>

#include <QDebug>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/render_window.hpp>

#include <vector>

using namespace rviz_common;
using namespace rviz_rendering;
using namespace rviz_default_plugins;

void MyShape::initialize(rviz_common::DisplayContext *context, rviz_common::properties::Property *parent) {
  context_ = context;
  shape_property_ = new rviz_common::properties::BoolProperty("MyShape", true, "", parent);
  connect(shape_property_, &properties::Property::changed, this, &MyShape::ColorChanged);

  // shape_ = std::make_shared<rviz_rendering::Shape>(Shape::Cone, context->getSceneManager());
  // handler_ = rviz_common::interaction::createSelectionHandler<MyShapeSelectionHandler>(shape_.get(), context);
  // handler_->addTrackedObjects(shape_->getRootNode());

  type_property_ = new properties::EnumProperty("shape_type", "Cone", "", shape_property_);
  type_property_->addOption("Cone", 0);
  type_property_->addOption("Cube", 1);
  type_property_->addOption("Cylinder", 2);
  type_property_->addOption("Sphere", 3);
  connect(type_property_, &properties::Property::changed, this, &MyShape::ShapeChanged);

  color_property_ = new properties::ColorProperty("color", Qt::red, "", shape_property_);
  connect(color_property_, &properties::Property::changed, this, &MyShape::ColorChanged);

  pos_property_ = new properties::VectorProperty("pos", Ogre::Vector3::ZERO, "", shape_property_);
  connect(pos_property_, &properties::Property::changed, this, &MyShape::ColorChanged);

  scale_property_ = new properties::VectorProperty("scale", Ogre::Vector3{0.1, 0.1, 0.1}, "", shape_property_);
  connect(scale_property_, &properties::Property::changed, this, &MyShape::ColorChanged);
}

void MyShape::update() {
  if (shape_changed_) {
    int type = type_property_->getOptionInt();
    shape_ = std::make_shared<rviz_rendering::Shape>(Shape::Type(type), context_->getSceneManager());
    handler_ = rviz_common::interaction::createSelectionHandler<MyShapeSelectionHandler>(shape_.get(), context_);
    handler_->addTrackedObjects(shape_->getRootNode());
    shape_changed_.store(false);
    changed_ = true;
  }
  if (changed_) {
    shape_->getRootNode()->setVisible(shape_property_->getBool());
    shape_->setColor(color_property_->getOgreColor());
    shape_->setPosition(pos_property_->getVector());
    shape_->setScale(scale_property_->getVector());
    changed_.store(false);
  }
}

rviz_common::interaction::V_AABB MyLineSelectionHandler::getAABBs(const rviz_common::interaction::Picked &obj) {
  rviz_common::interaction::V_AABB aabbs;
  for (auto handle : obj.extra_handles) {
    auto find_it = boxes_.find(Handles(obj.handle, handle - 1));
    if (find_it != boxes_.end()) {
      Ogre::WireBoundingBox *box = find_it->second.box;

      aabbs.push_back(box->getWorldBoundingBox(true)); // calculate bounding boxes if absent
    }
  }
  return aabbs;
}

void MyLineSelectionHandler::onSelect(const rviz_common::interaction::Picked &obj) {
  qDebug() << "MyLineSelectionHandler::onSelect size = " << obj.extra_handles.size();
  for (auto handle : obj.extra_handles) {
    uint64_t index = handleToIndex(handle);

    // sensor_msgs::msg::PointCloud2::ConstSharedPtr message = cloud_info_->message_;

    Eigen::Vector3f const &e_pt = my_line_->pts_[2 * index];
    Ogre::Vector3 pos = {e_pt.x(), e_pt.y(), e_pt.z()};
    pos = my_line_->lines_->getSceneNode()->convertLocalToWorldPosition(pos);

    float box_size_ = 0.004f;
    float size = box_size_ * 0.5f;

    Ogre::AxisAlignedBox aabb(pos - size, pos + size);

    createBox(Handles(obj.handle, index), aabb, "RVIZ/Cyan");
  }
}

void MyLineSelectionHandler::preRenderPass(uint32_t pass) {
  rviz_common::interaction::SelectionHandler::preRenderPass(pass);
  qDebug() << " MyLineSelectionHandler::preRenderPass pass = " << pass;
  switch (pass) {
  case 0:
    my_line_->setColorByPickHandler(rviz_common::interaction::SelectionManager::handleToColor(getHandle()));
    break;
  case 1:
    my_line_->setColorByIndex(true);
    break;
  default:
    break;
  }
}

void MyLineSelectionHandler::postRenderPass(uint32_t pass) {
  qDebug() << " MyLineSelectionHandler::postRenderPass pass = " << pass;

  rviz_common::interaction::SelectionHandler::postRenderPass(pass);

  if (pass == 1) { my_line_->setColorByIndex(false); }
}

void MyLine::initialize(rviz_common::DisplayContext *context, rviz_common::properties::Property *parent) {
  context_ = context;
  shape_property_ = new rviz_common::properties::BoolProperty("MyLine", true, "", parent);
  connect(shape_property_, &properties::Property::changed, this, &MyLine::ColorChanged);

  // shape_ = std::make_shared<rviz_rendering::Shape>(Shape::Cone, context->getSceneManager());
  // handler_ = rviz_common::interaction::createSelectionHandler<MyShapeSelectionHandler>(shape_.get(), context);
  // handler_->addTrackedObjects(shape_->getRootNode());

  type_property_ = new properties::EnumProperty("line_type", "line_strip", "", shape_property_);
  type_property_->addOption("line_strip", 0);
  type_property_->addOption("line_list", 1);
  connect(type_property_, &properties::Property::changed, this, &MyLine::ShapeChanged);

  color_property_ = new properties::ColorProperty("color", Qt::red, "", shape_property_);
  connect(color_property_, &properties::Property::changed, this, &MyLine::ColorChanged);

  pos_property_ = new properties::VectorProperty("pos", Ogre::Vector3::ZERO, "", shape_property_);
  connect(pos_property_, &properties::Property::changed, this, &MyLine::ColorChanged);

  width_property_ = new properties::FloatProperty("width", 0.01, "", shape_property_);
  connect(width_property_, &properties::Property::changed, this, &MyLine::ColorChanged);

  Eigen::Vector3f raw_pt{1, 1, 1};
  pts_ = {raw_pt * 0.1, raw_pt * 0.2, raw_pt * 0.3, raw_pt * 0.4};
}

void MyLine::FillPoints() {
  lines_->clear();

  uint32_t const counts = pts_.size();
  if (type_property_->getOptionInt() == 0) {
    lines_->setMaxPointsPerLine(counts);
    for (size_t i = 0; i < counts; i++) {
      Ogre::ColourValue c = color_property_->getOgreColor();

      Ogre::Vector3 v(pts_[i].x(), pts_[i].y(), pts_[i].z());
      lines_->addPoint(v, c);
    }
  }
  if (type_property_->getOptionInt() == 1) {
    lines_->setMaxPointsPerLine(2);
    lines_->setNumLines(static_cast<uint32_t>(counts / 2));

    for (size_t i = 0; i < counts / 2; i++) {
      Ogre::ColourValue c = color_property_->getOgreColor();
      // Ogre::ColourValue c = getColorForLine(i, c1);

      if (color_is_index_) {
        uint32_t color = (i + 1);
        c.a = 1.0f;
        c.r = ((color >> 16) & 0xff) / 255.0f;
        c.g = ((color >> 8) & 0xff) / 255.0f;
        c.b = (color & 0xff) / 255.0f;
      }

      Ogre::Vector3 v1(pts_[2 * i].x(), pts_[2 * i].y(), pts_[2 * i].z());
      Ogre::Vector3 v2(pts_[2 * i + 1].x(), pts_[2 * i + 1].y(), pts_[2 * i + 1].z());
      lines_->addPoint(v1, c);
      lines_->addPoint(v2, c);
      lines_->finishLine();
    }
  }
}

void MyLine::update() {
  if (shape_changed_) {
    if (!lines_) {
      lines_ = std::make_shared<rviz_rendering::BillboardLine>(context_->getSceneManager());
      handler_ = rviz_common::interaction::createSelectionHandler<MyLineSelectionHandler>(this, lines_.get(), context_);
      handler_->addTrackedObjects(lines_->getSceneNode());
    }
    // 添加点
    FillPoints();

    shape_changed_.store(false);
    changed_ = true;
  }
  if (changed_) {
    lines_->getSceneNode()->setVisible(shape_property_->getBool());
    auto color = color_property_->getOgreColor();
    lines_->setColor(color.r, color.g, color.b, color.a);
    lines_->setLineWidth(width_property_->getFloat());

    lines_->setPosition(pos_property_->getVector());

    changed_.store(false);
  }
}

void MyPointCloud::initialize(rviz_common::DisplayContext *context, rviz_common::properties::Property *parent) {
  using namespace rviz_default_plugins;
  context_ = context;
  shape_property_ = new rviz_common::properties::BoolProperty("MyPointCloud", true, "", parent);
  connect(shape_property_, &properties::Property::changed, this, &MyPointCloud::ColorChanged);

  scale_property_ = new properties::VectorProperty("scale", Ogre::Vector3{0.1, 0.1, 0.1}, "", shape_property_);
  connect(scale_property_, &properties::Property::changed, this, &MyPointCloud::ColorChanged);

  type_property_ = new properties::EnumProperty("shape_type", "RM_POINTS", "", shape_property_);
  type_property_->addOption("RM_POINTS", 0);
  type_property_->addOption("RM_SQUARES", 1);
  type_property_->addOption("RM_FLAT_SQUARES", 2);
  type_property_->addOption("RM_SPHERES", 3);
  type_property_->addOption("RM_TILES", 4);
  type_property_->addOption("RM_BOXES", 5);
  connect(type_property_, &properties::Property::changed, this, &MyPointCloud::ColorChanged);

  Eigen::Vector3f raw_pt{1, 1, 0};
  pts_ = {raw_pt, raw_pt * 2};

  // points
  std::vector<rviz_rendering::PointCloud::Point> pts;
  for (auto const &pt : pts_) {
    rviz_rendering::PointCloud::Point opt;
    opt.position = Ogre::Vector3(pt.x(), pt.y(), pt.z());
    opt.setColor(1, 0, 0);
    pts.emplace_back(opt);
  }

  cloud_ = std::make_shared<rviz_rendering::PointCloud>();
  cloud_->addPoints(pts.begin(), pts.end());
  selection_handler_ = interaction::createSelectionHandler<PointCloudSelectionHandler2>(0.004f, cloud_.get(), context_);
  cloud_->setPickColor(interaction::SelectionManager::handleToColor(selection_handler_->getHandle()));

  context_->getSceneManager()->getRootSceneNode()->attachObject(cloud_.get());
}

void MyPointCloud::update() {
  if (changed_) {
    cloud_->setVisible(shape_property_->getBool());
    //
    auto scale = scale_property_->getVector();
    cloud_->setDimensions(scale[0], scale[1], scale[2]);
    cloud_->setRenderMode(static_cast<PointCloud::RenderMode>(type_property_->getOptionInt()));

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

  line1_.initialize(context_, this);

  pc1_.initialize(context_, this);
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

  line1_.update();

  pc1_.update();
}

bool My3dDisplay::updateCamera() {
  // auto render_window = render_panel_->getRenderWindow();
  // auto proj_matrix = view_manager_->getCurrent()->getCamera()->getProjectionMatrix();
  // std::cout << " proj_matrix = " << proj_matrix << "\n";
  // rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_window)->setCustomProjectionMatrix(true, proj_matrix);
  return true;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(My3dDisplay, rviz_common::Display)