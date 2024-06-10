#pragma once

#include <QDebug>

#include <atomic>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>

#include <rviz/selection/selection_handler.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>



#include <Eigen/Dense>


class MyLine;

class MyLineSelectionHandler : public rviz::SelectionHandler {
  typedef std::set<uint64_t> S_int;

public:
  MyLineSelectionHandler(MyLine *my_line, rviz::BillboardLine *lines, rviz::DisplayContext *context)
      : rviz::SelectionHandler(context) {
    my_line_ = my_line;
    lines_ = lines;
  }

  // ~MyLineSelectionHandler() override{};
  void preRenderPass(uint32_t pass) override;
  void postRenderPass(uint32_t pass) override;
  bool needsAdditionalRenderPass(uint32_t pass) override;
  void onSelect(const rviz::Picked &obj) override;
  void getAABBs(const rviz::Picked& obj, rviz::V_AABB& aabbs) override;


  void createProperties(const rviz::Picked &obj, rviz::Property *parent_property) override {
    rviz::Property *group = new rviz::Property("lines ", QVariant(), "", parent_property);
    properties_.push_back(group);

    S_int indices = getIndicesOfSelectedPoints(obj);

    qDebug() << "createProperties, indices size = " << indices.size();

    for (auto index : indices) {
      qDebug() << "index == " << index << "\n";
    }

    group->expand();
  };

  uint64_t handleToIndex(uint64_t handle) const { return (handle & 0xffffffff) - 1; }

  S_int getIndicesOfSelectedPoints(const rviz::Picked &obj) {
    S_int indices;
    for (auto handle : obj.extra_handles) {
      indices.insert(handleToIndex(handle));
    }
    return indices;
  }

  void updateProperties() override{};

private:
  rviz::BillboardLine *lines_;
  MyLine *my_line_;

  rviz::VectorProperty *position_property_;
  rviz::ColorProperty *color_property_;
};

class MyLine : public QObject {
public:
  void initialize(rviz::DisplayContext *context, rviz::Property *parent);

  void update();

public:
  rviz::DisplayContext *context_ = nullptr;
  std::shared_ptr<rviz::BillboardLine> lines_;
  std::shared_ptr<MyLineSelectionHandler> handler_ = nullptr;
  //
  std::atomic_bool changed_ = {true};
  std::atomic_bool shape_changed_ = {true};
  rviz::ColorProperty *color_property_;
  rviz::EnumProperty *type_property_;
  rviz::BoolProperty *shape_property_;
  rviz::VectorProperty *pos_property_;
  rviz::FloatProperty *width_property_;
  void ColorChanged() { changed_.store(true); };
  void ShapeChanged() { shape_changed_.store(true); }
  std::vector<Eigen::Vector3f> pts_;

public:
  void setColorByIndex(bool set) {
    qDebug() << "setColorByIndex set = " << set;

    color_is_index_ = set;
    FillPoints();

    // for (auto &renderable : lines_->getChains()) {
    //   renderable->setCustomParameter(RVIZ_RENDERING_PICK_COLOR_PARAMETER, pick_col);
    // }
    // shape_changed_.store(true);
  };
  void setColorByPickHandler(const Ogre::ColourValue &color) {
    color_is_index_ = false;
    pick_color_ = color;
    Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);

    // FillPoints();
    // std::cout << "setColorByPickHandler lines_->getChains() size = " << lines_->getChains().size()
    //           << " pick_color_ = " << pick_color_ << "\n";
    // for (auto &renderable : lines_->getChains()) {
    //   renderable->setCustomParameter(RVIZ_RENDERING_PICK_COLOR_PARAMETER, pick_col);
    // }
    // shape_changed_.store(true);
  }
  bool color_is_index_ = false;
  Ogre::ColourValue pick_color_;

  // uint32_t getColorForLine(uint32_t pt_index, Ogre::ColourValue const &c2) const {
  //   uint32_t color;
  //   auto root = Ogre::Root::getSingletonPtr();

  //   if (color_is_index_) {
  //     // convert to ColourValue, so we can then convert to the rendersystem-specific color type
  //     color = (pt_index + 1);
  //     Ogre::ColourValue c;
  //     c.a = 1.0f;
  //     c.r = ((color >> 16) & 0xff) / 255.0f;
  //     c.g = ((color >> 8) & 0xff) / 255.0f;
  //     c.b = (color & 0xff) / 255.0f;
  //     root->convertColourValue(c, &color);
  //   } else {
  //     root->convertColourValue(c2, &color);
  //   }
  //   return color;
  // }

private:
  void FillPoints();
};