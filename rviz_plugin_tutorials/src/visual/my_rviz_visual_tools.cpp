#include "visual/my_rviz_visual_tools.hpp"

#include "markers/arrow_marker.h"
#include "markers/my_marker_selection_handler.h"

#include <Eigen/Geometry>

#include <rviz/properties/color_property.h>

using namespace rviz_visual_tools;

std_msgs::ColorRGBA qcolorToColorRGBA(const QColor &qcolor) {
  std_msgs::ColorRGBA color_rgba;
  color_rgba.r = qcolor.redF();   // redF() 返回一个介于0到1之间的浮点数
  color_rgba.g = qcolor.greenF(); // greenF() 返回一个介于0到1之间的浮点数
  color_rgba.b = qcolor.blueF();  // blueF() 返回一个介于0到1之间的浮点数
  color_rgba.a = qcolor.alphaF(); // alphaF() 返回一个介于0到1之间的浮点数
  return color_rgba;
}

MyRvizVisualTools::MyRvizVisualTools(std::string base_frame) : RvizVisualTools(base_frame) {
  this->enableBatchPublishing();
}

MyRvizVisualTools::~MyRvizVisualTools() { reset(); }

void MyRvizVisualTools::initialize(rviz::Display *parent, rviz::DisplayContext *context) {
  parent_ = parent;
  context_ = context;
  root_node_ = parent_->getSceneNode();

  color_root_ = new rviz::ColorProperty("root color", Qt::gray, "new color to all", parent_);

  ns_root_ = new rviz::BoolProperty("ns filter", true, "null", parent_);

  root_node_->getUserObjectBindings().setUserAny(Ogre::Any(parent));
}

void MyRvizVisualTools::reset_ns(std::string const &ns) {
  if (ns_properties_.find(ns) == ns_properties_.end()) { return; }
  for (auto *single_node : ns_filted_node_[ns]) {
    all_scene_node_.erase(single_node);                        // remove mark
    root_node_->removeAndDestroyChild(single_node->getName()); // remove scene node
  }
  ns_filted_node_.erase(ns);
}

void MyRvizVisualTools::reset() {
  for (auto const &[name, props] : ns_properties_) {
    reset_ns(name);
  }
  ns_root_->removeChildren(0, ns_root_->numChildren());
  ns_properties_.clear();
}

void MyRvizVisualTools::updateColor(std::string const &ns, QColor const &color) {
  if (ns_properties_.find(ns) == ns_properties_.end()) { return; }

  auto const new_color = qcolorToColorRGBA(color);

  markers_.markers.reserve(ns_filted_node_[ns].size());

  beginInit();

  for (auto *single_node : ns_filted_node_[ns]) {
    if (auto *mark_view = all_scene_node_[single_node]; mark_view) {
      auto new_mark = *mark_view->getMessage();
      new_mark.color = new_color;
      for (auto &old_color : new_mark.colors) {
        old_color = new_color;
      }

      markers_.markers.push_back(new_mark);
    }
  }

  endInit();
}

rviz::MarkerBase *MyRvizVisualTools::CreateMarkView(visualization_msgs::Marker const &mark) {
  auto *new_scene_node = root_node_->createChildSceneNode();

  auto mark_view = rviz::createMarker2(mark.type, nullptr, context_, new_scene_node);
  mark_view->setMessage(visualization_msgs::MarkerPtr(new visualization_msgs::Marker(std::move(mark))));

  ns_filted_node_[mark.ns].push_back(new_scene_node);
  all_scene_node_[new_scene_node] = mark_view;

  // 新的ns
  if (ns_properties_.find(mark.ns) == ns_properties_.end()) {
    // 更新筛选器
    auto new_prop = new rviz::BoolProperty(QString::fromStdString(mark.ns), true, "null", ns_root_);
    auto new_color = new rviz::ColorProperty("color", Qt::gray, "new color", new_prop);
    connect(new_color, &rviz::ColorProperty::changed, [this, new_prop, new_color] {
      updateColor(new_prop->getNameStd(), new_color->getColor());
      reset_ns(new_prop->getNameStd());
    });
    connect(color_root_, &rviz::ColorProperty::changed, [this, new_color]() {
      new_color->setColor(color_root_->getColor());
    });

    // 响应筛选
    connect(new_prop, &rviz::BoolProperty::changed, [this, new_prop]() {
      for (auto &single_node : ns_filted_node_[new_prop->getNameStd()]) {
        single_node->setVisible(new_prop->getBool());
      }
    });
    connect(ns_root_, &rviz::Property::changed, [this, new_prop]() { new_prop->setBool(ns_root_->getBool()); });

    ns_properties_[mark.ns] = new_prop;
  }

  //
}

void MyRvizVisualTools::update() {
  if (!inited_ || markers_.markers.empty()) { return; }
  for (auto const &mark : markers_.markers) {
    CreateMarkView(mark);
  }
  markers_.markers.clear();
}

bool MyRvizVisualTools::publishRect(Eigen::Vector3d const &pos,
                                    Eigen::Vector3d const &normal,
                                    colors color,
                                    double x_width,
                                    double y_width,
                                    std::string const ns) {
  using namespace Eigen;
  Eigen::Isometry3d Twl = Translation3d(pos) * Quaterniond::FromTwoVectors(Vector3d::UnitZ(), normal);
  // 左上角点开始顺时针4个点
  Vector3d const pt0 = Vector3d{-1, 1, 0};
  Vector3d const pt1 = Vector3d{1, 1, 0};
  Vector3d const pt2 = Vector3d{1, -1, 0};
  Vector3d const pt3 = Vector3d{-1, -1, 0};
  //
  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.ns = ns;
  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = convertPose(Twl);

  triangle_marker_.scale.x = x_width;
  triangle_marker_.scale.y = y_width;
  triangle_marker_.scale.z = 1.0;

  triangle_marker_.points.clear();
  triangle_marker_.points.reserve(6);
  triangle_marker_.points.emplace_back(convertPoint(pt0));
  triangle_marker_.points.emplace_back(convertPoint(pt3));
  triangle_marker_.points.emplace_back(convertPoint(pt2));

  triangle_marker_.points.emplace_back(convertPoint(pt2));
  triangle_marker_.points.emplace_back(convertPoint(pt1));
  triangle_marker_.points.emplace_back(convertPoint(pt0));

  return publishMarker(triangle_marker_);
}

bool MyRvizVisualTools::publishCircle(Eigen::Vector3d const &pos,
                                      Eigen::Vector3d const &normal,
                                      double const radius_m,
                                      rviz_visual_tools::colors color,
                                      std::string const ns) {
  double const length_m = 2 * radius_m * sin(M_PI / 36);
  return publishRegularPolygon(pos, normal, 36, length_m, color, ns);
}

bool MyRvizVisualTools::publishRegularTriangle(Eigen::Vector3d const &pos,
                                               Eigen::Vector3d const &normal,
                                               double const length_m,
                                               rviz_visual_tools::colors color,
                                               std::string const ns) {
  return publishRegularPolygon(pos, normal, 3, length_m, color, ns);
}

bool MyRvizVisualTools::publishRegularPolygon(Eigen::Vector3d const &pos,
                                              Eigen::Vector3d const &normal,
                                              int const line_count,  // 多边形的边数
                                              double const length_m, // 多边形的边长
                                              rviz_visual_tools::colors color,
                                              std::string const ns) {
  using namespace Eigen;
  Eigen::Isometry3d Twl = Translation3d(pos) * Quaterniond::FromTwoVectors(Vector3d::UnitZ(), normal);
  Vector3d const origin = Vector3d::Zero();
  //
  triangle_marker_.header.stamp = ros::Time::now();
  triangle_marker_.id++;

  triangle_marker_.ns = ns;
  triangle_marker_.color = getColor(color);
  triangle_marker_.pose = convertPose(Twl);
  //
  double const length2 = length_m / 2 / sin(M_PI / line_count);
  triangle_marker_.scale.x = length2;
  triangle_marker_.scale.y = length2;
  triangle_marker_.scale.z = 1.0;
  //
  triangle_marker_.points.clear();
  size_t const counts = line_count;
  triangle_marker_.points.reserve(counts * 3);
  double const d_theta = 1.0 / counts * 2 * M_PI;
  double const end = 2 * M_PI;
  double const start = (counts % 2 == 0 ? 0.5 * d_theta : 0);
  for (double theta = start; theta < end; theta += d_theta) {
    double const theta2 = theta + d_theta;
    triangle_marker_.points.emplace_back(convertPoint(Vector3d{-sin(theta), cos(theta), 0.0}));
    triangle_marker_.points.emplace_back(convertPoint(Vector3d{-sin(theta2), cos(theta2), 0.0}));
    triangle_marker_.points.emplace_back(geometry_msgs::Point{});
  }
  return publishMarker(triangle_marker_);
}

bool MyRvizVisualTools::publishRegularPolygon2(Eigen::Vector3d const &pos,
                                               Eigen::Vector3d const &normal,
                                               int const line_count,  // 多边形的边数
                                               double const radius_m, // 多边形的边长
                                               rviz_visual_tools::colors color,
                                               std::string const ns) {
  double const length_m = 2 * radius_m * sin(M_PI / line_count);
  return publishRegularPolygon(pos, normal, line_count, length_m, color, ns);
}