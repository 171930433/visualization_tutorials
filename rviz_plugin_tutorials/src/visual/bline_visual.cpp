#include "visual/bline_visual.hpp"

#include <OgreRoot.h>
#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

#include <rviz/selection/selection_manager.h>

using namespace rviz;

void MyLineSelectionHandler::getAABBs(const rviz::Picked &obj, rviz::V_AABB &aabbs) {
  if (my_line_ && my_line_->type_property_->getOptionInt() == 0) {
    return rviz::SelectionHandler::getAABBs(obj, aabbs);
  }

  for (auto handle : obj.extra_handles) {
    auto find_it = boxes_.find({obj.handle, handle - 1});
    if (find_it != boxes_.end()) {
      Ogre::WireBoundingBox *box = find_it->second.second;

      aabbs.push_back(box->getWorldBoundingBox(true)); // calculate bounding boxes if absent
    }
  }
}

void MyLineSelectionHandler::onSelect(const rviz::Picked &obj) {
  if (my_line_ && my_line_->type_property_->getOptionInt() == 0) { return rviz::SelectionHandler::onSelect(obj); }

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

    createBox({obj.handle, index}, aabb, "RVIZ/Cyan");
  }
}

void MyLineSelectionHandler::preRenderPass(uint32_t pass) {
  rviz::SelectionHandler::preRenderPass(pass);
  qDebug() << " MyLineSelectionHandler::preRenderPass pass = " << pass << " getHandle() = " << getHandle();

  switch (pass) {
  case 0:
    my_line_->setColorByPickHandler(rviz::SelectionManager::handleToColor(getHandle()));
    break;
  case 1:
    my_line_->setColorByIndex(true);
    break;
  default:
    break;
  }
}

void MyLineSelectionHandler::postRenderPass(uint32_t pass) {
  rviz::SelectionHandler::postRenderPass(pass);
  qDebug() << " MyLineSelectionHandler::preRenderPass pass = " << pass << " getHandle() = " << getHandle();

  if (pass == 1) { my_line_->setColorByIndex(false); }
}

bool MyLineSelectionHandler::needsAdditionalRenderPass(uint32_t pass) {
  // 线段只能直接选择
  if (my_line_ && my_line_->type_property_->getOptionInt() == 0) {
    return rviz::SelectionHandler::needsAdditionalRenderPass(pass);
  }
  return pass < 2;
}

void MyLine::initialize(rviz::DisplayContext *context, rviz::Property *parent) {
  context_ = context;
  shape_property_ = new rviz::BoolProperty("MyLine", true, "", parent);
  connect(shape_property_, &rviz::Property::changed, this, &MyLine::ColorChanged);

  // shape_ = std::make_shared<rviz::Shape>(Shape::Cone, context->getSceneManager());
  // handler_ = rviz::createSelectionHandler<MyShapeSelectionHandler>(shape_.get(), context);
  // handler_->addTrackedObjects(shape_->getRootNode());

  type_property_ = new rviz::EnumProperty("line_type", "line_list", "", shape_property_);
  type_property_->addOption("line_strip", 0);
  type_property_->addOption("line_list", 1);
  connect(type_property_, &rviz::Property::changed, this, &MyLine::ShapeChanged);

  color_property_ = new rviz::ColorProperty("color", Qt::red, "", shape_property_);
  connect(color_property_, &rviz::Property::changed, this, &MyLine::ColorChanged);

  pos_property_ = new rviz::VectorProperty("pos", Ogre::Vector3::ZERO, "", shape_property_);
  connect(pos_property_, &rviz::Property::changed, this, &MyLine::ColorChanged);

  width_property_ = new rviz::FloatProperty("width", 0.1, "", shape_property_);
  connect(width_property_, &rviz::Property::changed, this, &MyLine::ColorChanged);

  Eigen::Vector3f raw_pt{1, 1, 0};
  pts_ = {raw_pt * 0.2, raw_pt * 0.4, raw_pt * 0.6, raw_pt * 0.8};
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
    auto root = Ogre::Root::getSingletonPtr();

    for (uint32_t i = 0; i < counts / 2; i++) {
      if (i != 0) { lines_->newLine(); }
      Ogre::ColourValue c = color_property_->getOgreColor();
      // Ogre::ColourValue c = getColorForLine(i, c1);

      if (color_is_index_) {
        uint32_t color = (i + 1);
        c.a = 1.0f;
        c.r = ((color >> 16) & 0xff) / 255.0f;
        c.g = ((color >> 8) & 0xff) / 255.0f;
        c.b = (color & 0xff) / 255.0f;
      }

      uint32_t color2;
      root->convertColourValue(c, &color2);
      qDebug() << "                      color2 = " << color2;

      Ogre::Vector3 v1(pts_[2 * i].x(), pts_[2 * i].y(), pts_[2 * i].z());
      Ogre::Vector3 v2(pts_[2 * i + 1].x(), pts_[2 * i + 1].y(), pts_[2 * i + 1].z());
      lines_->addPoint(v1, c);
      lines_->addPoint(v2, c);
    }
  }
  if(lines_->getSceneNode())
  {
    lines_->getSceneNode()->needUpdate();
  }
}

void MyLine::update() {
  if (shape_changed_) {
    if (!lines_) {
      lines_ = std::make_shared<rviz::BillboardLine>(context_->getSceneManager());
      handler_ = std::make_shared<MyLineSelectionHandler>(this, lines_.get(), context_);
      handler_->addTrackedObjects(lines_->getSceneNode());
      qDebug() << " MyLine::update() handler_" << handler_->getHandle();
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
