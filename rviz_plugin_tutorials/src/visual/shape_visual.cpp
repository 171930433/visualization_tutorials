#include "visual/shape_visual.hpp"

#include <OgreSceneNode.h>

using namespace rviz;

void ShapeVisual::initialize(rviz::DisplayContext *context, rviz::Property *parent) {
  context_ = context;
  shape_property_ = new rviz::BoolProperty("ShapeVisual", true, "", parent);
  connect(shape_property_, &Property::changed, this, &ShapeVisual::ColorChanged);

  // shape_ = std::make_shared<rviz::Shape>(Shape::Cone, context->getSceneManager());
  // handler_ = rviz::interaction::createSelectionHandler<MyShapeSelectionHandler>(shape_.get(), context);
  // handler_->addTrackedObjects(shape_->getRootNode());

  type_property_ = new rviz::EnumProperty("shape_type", "Cone", "", shape_property_);
  type_property_->addOption("Cone", 0);
  type_property_->addOption("Cube", 1);
  type_property_->addOption("Cylinder", 2);
  type_property_->addOption("Sphere", 3);
  connect(type_property_, &Property::changed, this, &ShapeVisual::ShapeChanged);

  color_property_ = new rviz::ColorProperty("color", Qt::red, "", shape_property_);
  connect(color_property_, &Property::changed, this, &ShapeVisual::ColorChanged);

  pos_property_ = new rviz::VectorProperty("pos", Ogre::Vector3::ZERO, "", shape_property_);
  connect(pos_property_, &Property::changed, this, &ShapeVisual::ColorChanged);

  scale_property_ = new rviz::VectorProperty("scale", Ogre::Vector3{0.1, 0.1, 0.1}, "", shape_property_);
  connect(scale_property_, &Property::changed, this, &ShapeVisual::ColorChanged);
}

void ShapeVisual::update() {
  if (shape_changed_) {
    int type = type_property_->getOptionInt();
    shape_ = std::make_shared<rviz::Shape>(Shape::Type(type), context_->getSceneManager());
    handler_ = std::make_shared<MyShapeSelectionHandler>(shape_.get(), context_);
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