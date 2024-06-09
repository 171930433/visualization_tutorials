#pragma once

#include <QObject>
#include <atomic>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>

#include "rviz_common/properties/parse_color.hpp"
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/vector_property.hpp>

#include <Eigen/Dense>
#include <OgreBillboardChain.h>
#include <OgreEntity.h>
#include <OgreRoot.h>
#include <QDebug>

#include <rviz_rendering/custom_parameter_indices.hpp>

namespace rviz_common {
class Display;
class RenderPanel;
class VisualizationManager;
class ViewManager;

} // namespace rviz_common

class MyShapeSelectionHandler : public rviz_common::interaction::SelectionHandler {
public:
  MyShapeSelectionHandler(rviz_rendering::Shape *shape, rviz_common::DisplayContext *context)
      : rviz_common::interaction::SelectionHandler(context), shape_(shape) {}

  ~MyShapeSelectionHandler() override{};

  void createProperties(const rviz_common::interaction::Picked &obj,
                        rviz_common::properties::Property *parent_property) override {
    (void)obj;

    rviz_common::properties::Property *group =
        new rviz_common::properties::Property("Shape ", QVariant(), "", parent_property);
    properties_.push_back(group);

    position_property_ = new rviz_common::properties::VectorProperty("Position", shape_->getPosition(), "", group);
    position_property_->setReadOnly(true);

    group->expand();
  };

  void updateProperties() override{};

private:
  rviz_rendering::Shape *shape_ = nullptr;
  rviz_common::properties::VectorProperty *position_property_;
  rviz_common::properties::ColorProperty *color_property_;

  template <typename T, typename... Args>
  friend typename std::shared_ptr<T> rviz_common::interaction::createSelectionHandler(Args... arguments);
};

class MyShape : public QObject {
public:
  void initialize(rviz_common::DisplayContext *context, rviz_common::properties::Property *parent);

  void update();

public:
  rviz_common::DisplayContext *context_ = nullptr;
  std::shared_ptr<rviz_rendering::Shape> shape_ = nullptr;
  std::shared_ptr<MyShapeSelectionHandler> handler_ = nullptr;
  //
  std::atomic_bool changed_ = {true};
  std::atomic_bool shape_changed_ = {true};
  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::EnumProperty *type_property_;
  rviz_common::properties::BoolProperty *shape_property_;
  rviz_common::properties::VectorProperty *pos_property_;
  rviz_common::properties::VectorProperty *scale_property_;
  void ColorChanged() { changed_.store(true); };
  void ShapeChanged() { shape_changed_.store(true); }
};

class MyLine;

class MyLineSelectionHandler : public rviz_common::interaction::SelectionHandler {
  typedef std::set<uint64_t> S_int;

public:
  MyLineSelectionHandler(MyLine *my_line, rviz_rendering::BillboardLine *lines, rviz_common::DisplayContext *context)
      : rviz_common::interaction::SelectionHandler(context) {
    my_line_ = my_line;
    lines_ = lines;
  }

  ~MyLineSelectionHandler() override{};
  void preRenderPass(uint32_t pass) override;
  void postRenderPass(uint32_t pass) override;
  bool needsAdditionalRenderPass(uint32_t pass) override { return pass < 2; }
  void onSelect(const rviz_common::interaction::Picked & obj) override;
  rviz_common::interaction::V_AABB getAABBs(const rviz_common::interaction::Picked & obj) override;

  void createProperties(const rviz_common::interaction::Picked &obj,
                        rviz_common::properties::Property *parent_property) override {
    rviz_common::properties::Property *group =
        new rviz_common::properties::Property("lines ", QVariant(), "", parent_property);
    properties_.push_back(group);

    S_int indices = getIndicesOfSelectedPoints(obj);

    qDebug() << "createProperties, indices size = " << indices.size();

    for (auto index : indices) {
      qDebug() << "index == " << index << "\n";
    }

    group->expand();
  };

  uint64_t handleToIndex(uint64_t handle) const { return (handle & 0xffffffff) - 1; }

  S_int getIndicesOfSelectedPoints(const rviz_common::interaction::Picked &obj) {
    S_int indices;
    for (auto handle : obj.extra_handles) {
      indices.insert(handleToIndex(handle));
    }
    return indices;
  }

  void updateProperties() override{};

private:
  rviz_rendering::BillboardLine *lines_;
  MyLine *my_line_;

  rviz_common::properties::VectorProperty *position_property_;
  rviz_common::properties::ColorProperty *color_property_;

  template <typename T, typename... Args>
  friend typename std::shared_ptr<T> rviz_common::interaction::createSelectionHandler(Args... arguments);
};

class MyLine : public QObject {
public:
  void initialize(rviz_common::DisplayContext *context, rviz_common::properties::Property *parent);

  void update();

public:
  rviz_common::DisplayContext *context_ = nullptr;
  std::shared_ptr<rviz_rendering::BillboardLine> lines_;
  std::shared_ptr<MyLineSelectionHandler> handler_ = nullptr;
  //
  std::atomic_bool changed_ = {true};
  std::atomic_bool shape_changed_ = {true};
  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::EnumProperty *type_property_;
  rviz_common::properties::BoolProperty *shape_property_;
  rviz_common::properties::VectorProperty *pos_property_;
  rviz_common::properties::FloatProperty *width_property_;
  void ColorChanged() { changed_.store(true); };
  void ShapeChanged() { shape_changed_.store(true); }
  std::vector<Eigen::Vector3f> pts_;

public:
  void setColorByIndex(bool set) {
    qDebug() << "setColorByIndex set = " << set;

    color_is_index_ = set;
    FillPoints();

    // shape_changed_.store(true);
  };
  void setColorByPickHandler(const Ogre::ColourValue &color) {
    pick_color_ = color;
    Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);

    FillPoints();
    qDebug() << "setColorByPickHandler lines_->getChains() size = " << lines_->getChains().size();
    for (auto &renderable : lines_->getChains()) {
      renderable->setCustomParameter(RVIZ_RENDERING_PICK_COLOR_PARAMETER, pick_col);
    }
    // shape_changed_.store(true);
  }
  bool color_is_index_ = false;
  Ogre::ColourValue pick_color_;

  uint32_t getColorForLine(uint32_t pt_index, Ogre::ColourValue const &c2) const {
    uint32_t color;
    auto root = Ogre::Root::getSingletonPtr();

    if (color_is_index_) {
      // convert to ColourValue, so we can then convert to the rendersystem-specific color type
      color = (pt_index + 1);
      Ogre::ColourValue c;
      c.a = 1.0f;
      c.r = ((color >> 16) & 0xff) / 255.0f;
      c.g = ((color >> 8) & 0xff) / 255.0f;
      c.b = (color & 0xff) / 255.0f;
      root->convertColourValue(c, &color);
    } else {
      root->convertColourValue(c2, &color);
    }
    return color;
  }

private:
  void FillPoints();
};

class My3dDisplay : public rviz_common::Display {
public:
  My3dDisplay();

  void initialize(rviz_common::DisplayContext *context) override;

protected:
  void setupRenderPanel();
  void update(float wall_dt, float ros_dt) override;
  bool updateCamera();

protected:
  std::shared_ptr<rviz_common::VisualizationManager> manager_;
  std::shared_ptr<rviz_common::RenderPanel> render_panel_;
  std::shared_ptr<rviz_common::ViewManager> view_manager_;
  std::atomic_bool inited_ = {false};

  // 最佳自定义图元绘制
protected:
protected:
  MyShape shape2_;
  MyShape shape3_;

  MyLine line1_;
};