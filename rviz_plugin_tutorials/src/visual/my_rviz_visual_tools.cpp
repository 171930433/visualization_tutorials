#include "visual/my_rviz_visual_tools.hpp"

#include "markers/arrow_marker.h"
#include "markers/my_marker_selection_handler.h"

#include <Eigen/Geometry>

using namespace rviz_visual_tools;

void MyRvizVisualTools::initialize(rviz::Display *parent, rviz::DisplayContext *context) {
  parent_ = parent;
  context_ = context;
  scene_node_ = parent_->getSceneNode();

  ns_root_ = new rviz::BoolProperty("ns filter", true, "null", parent_);

  // std::function<void()> cbk = [parent]() { parent->setTopic("", ""); };

  // Ogre::Any any_cbk(std::make_shared<std::function<void()>>(cbk));
  scene_node_->getUserObjectBindings().setUserAny(Ogre::Any(parent));
}

rviz::MarkerBase *MyRvizVisualTools::CreateMarkView(visualization_msgs::Marker const &mark) {
  auto *new_scene_node = scene_node_->createChildSceneNode();
  // new_scene_node->setUserAny();

  auto mark_view = rviz::createMarker2(mark.type, nullptr, context_, new_scene_node);
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

bool MyRvizVisualTools::publishPlaneRect(Eigen::Vector3d const &pos,
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

bool MyRvizVisualTools::publishPlaneCircle(Eigen::Vector3d const &pos,
                                           Eigen::Vector3d const &normal,
                                           double const radius_m,
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

  triangle_marker_.scale.x = radius_m;
  triangle_marker_.scale.y = radius_m;
  triangle_marker_.scale.z = 1.0;

  triangle_marker_.points.clear();
  size_t counts = 36;
  triangle_marker_.points.reserve(counts * 3);
  double const d_theta = 1.0 / counts * 2 * M_PI;
  double const end = 2 * M_PI;
  for (double theta = 0; theta < end; theta += d_theta) {
    double const theta2 = theta + d_theta;
    triangle_marker_.points.emplace_back(convertPoint(Vector3d{cos(theta), sin(theta), 0.0}));
    triangle_marker_.points.emplace_back(convertPoint(Vector3d{cos(theta2), sin(theta2), 0.0}));
    triangle_marker_.points.emplace_back(geometry_msgs::Point{});
  }
  return publishMarker(triangle_marker_);
}

bool MyRvizVisualTools::publishPlaneTriangle(Eigen::Vector3d const &pos,
                                             Eigen::Vector3d const &normal,
                                             double const length_m,
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

  // 外切圆半径
  double const length2 = length_m / 0.5 * sin(30.0 / 180 * M_PI);
  triangle_marker_.scale.x = length2;
  triangle_marker_.scale.y = length2;
  triangle_marker_.scale.z = 1.0;

  triangle_marker_.points.clear();
  triangle_marker_.points.reserve(3);
  double theta = 0;
  double const d_theta = 120.0 / 180 * M_PI;
  triangle_marker_.points.emplace_back(convertPoint(Vector3d{cos(theta), sin(theta), 0.0}));
  theta += d_theta;
  triangle_marker_.points.emplace_back(convertPoint(Vector3d{cos(theta), sin(theta), 0.0}));
  theta += d_theta;
  triangle_marker_.points.emplace_back(convertPoint(Vector3d{cos(theta), sin(theta), 0.0}));

  return publishMarker(triangle_marker_);
}

bool MyRvizVisualTools::publishPlaneRegularPolygon(Eigen::Vector3d const &pos,
                                                   Eigen::Vector3d const &normal,
                                                   int const line_count,  // 多边形的边数
                                                   double const radius_m, // 外切圆半径
                                                   rviz_visual_tools::colors color,
                                                   std::string const ns) {
                                                    return true;
                                                   }