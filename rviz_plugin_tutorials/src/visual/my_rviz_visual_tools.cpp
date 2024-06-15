#include "visual/my_rviz_visual_tools.hpp"


void MyRvizVisualTools::initialize(rviz::Display *parent, rviz::DisplayContext *context) {
  parent_ = parent;
  context_ = context;
  scene_node_ = parent_->getSceneNode();

  ns_root_ = new rviz::BoolProperty("ns filter", true, "null", parent_);
}

rviz::MarkerBase *MyRvizVisualTools::CreateMarkView(visualization_msgs::Marker const &mark) {
  auto *new_scene_node = scene_node_->createChildSceneNode();

  auto mark_view = rviz::createMarker(mark.type, nullptr, context_, new_scene_node);
  mark_view->setMessage(mark);

  ns_filted_node_[mark.ns].push_back(new_scene_node);
  all_scene_node_[mark_view] = new_scene_node;

  // 新的ns
  if (ns_properties_.find(mark.ns) == ns_properties_.end()) {
    // 更新筛选器
    auto new_prop = new rviz::BoolProperty(QString::fromStdString(mark.ns), true, "null", ns_root_);

    // 响应筛选
    connect(new_prop, &rviz::BoolProperty::changed, [this, new_prop]() {
      for (auto single_node : this->ns_filted_node_[new_prop->getNameStd()]) {
        single_node->setVisible(new_prop->getBool());
      }
    });

    ns_properties_[mark.ns] = new_prop;
  }

  //
}

void MyRvizVisualTools::update() {
  if (markers_.markers.empty()) { return; }
  for (auto const &mark : markers_.markers) {
    CreateMarkView(mark);
  }
  markers_.markers.clear();
}