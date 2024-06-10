#pragma once

#include <atomic>

#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>

#include <rviz/selection/selection_handler.h>
#include <rviz/selection/selection_handler.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/shape.h>


class MyShapeSelectionHandler : public rviz::SelectionHandler {
public:
  MyShapeSelectionHandler(rviz::Shape *shape, rviz::DisplayContext *context)
      : rviz::SelectionHandler(context), shape_(shape) {}

  ~MyShapeSelectionHandler() override{};

  void createProperties(const rviz::Picked &obj,
                        rviz::Property *parent_property) override {
    (void)obj;

    rviz::Property *group =
        new rviz::Property("Shape ", QVariant(), "", parent_property);
    properties_.push_back(group);

    position_property_ = new rviz::VectorProperty("Position", shape_->getPosition(), "", group);
    position_property_->setReadOnly(true);

    group->expand();
  };

  void updateProperties() override{};

private:
  rviz::Shape *shape_ = nullptr;
  rviz::VectorProperty *position_property_;
  rviz::ColorProperty *color_property_;
};

class ShapeVisual : public QObject {
public:
  void initialize(rviz::DisplayContext *context, rviz::Property *parent);

  void update();

public:
  rviz::DisplayContext *context_ = nullptr;
  std::shared_ptr<rviz::Shape> shape_ = nullptr;
  std::shared_ptr<MyShapeSelectionHandler> handler_ = nullptr;
  //
  std::atomic_bool changed_ = {true};
  std::atomic_bool shape_changed_ = {true};
  rviz::ColorProperty *color_property_;
  rviz::EnumProperty *type_property_;
  rviz::BoolProperty *shape_property_;
  rviz::VectorProperty *pos_property_;
  rviz::VectorProperty *scale_property_;
  void ColorChanged() { changed_.store(true); };
  void ShapeChanged() { shape_changed_.store(true); }
};