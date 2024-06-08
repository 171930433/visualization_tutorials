#pragma once

#include <QObject>
#include <atomic>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include <rviz_rendering/objects/shape.hpp>

#include "rviz_common/properties/parse_color.hpp"
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/vector_property.hpp>

#include <OgreEntity.h>

namespace rviz_common {
class Display;
class RenderPanel;
class VisualizationManager;
class ViewManager;

} // namespace rviz_common

class MySelectionHandler : public rviz_common::interaction::SelectionHandler {
public:
  MySelectionHandler(rviz_rendering::Shape *shape, rviz_common::DisplayContext *context)
      : rviz_common::interaction::SelectionHandler(context), shape_(shape) {}

  ~MySelectionHandler() override{};

  void createProperties(const rviz_common::interaction::Picked &obj,
                        rviz_common::properties::Property *parent_property) override {
    (void)obj;

    rviz_common::properties::Property *group =
        new rviz_common::properties::Property("Shape ", QVariant(), "", parent_property);
    properties_.push_back(group);

    position_property_ = new rviz_common::properties::VectorProperty("Position", shape_->getPosition(), "", group);
    position_property_->setReadOnly(true);

    // auto* color = Ogre::any_cast<rviz_common::properties::ColorProperty*>(shape_->getEntity()->getUserObjectBindings().getUserAny());
    // color_property_ = new rviz_common::properties::ColorProperty("color", color->getColor(), "", group);
    // color_property_->setReadOnly(true);

    group->expand();
  };

  void updateProperties() override{
      // auto* color = Ogre::any_cast<rviz_common::properties::ColorProperty*>(shape_->getEntity()->getUserObjectBindings().getUserAny());

      // color_property_->setColor(color->getColor());
  };

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
  std::shared_ptr<MySelectionHandler> handler_ = nullptr;
  //
  std::atomic_bool changed_ = {false};
  std::atomic_bool shape_changed_ = {false};
  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::EnumProperty *type_property_;
  rviz_common::properties::BoolProperty *shape_property_;
  rviz_common::properties::VectorProperty *pos_property_;
  void ColorChanged() { changed_.store(true); };
  void ShapeChanged() { shape_changed_.store(true); }
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
};