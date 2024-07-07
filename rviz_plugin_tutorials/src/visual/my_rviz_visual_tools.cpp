#include "visual/my_rviz_visual_tools.hpp"

#include "markers/arrow_marker.h"
#include "markers/my_marker_selection_handler.h"

#include <Eigen/Geometry>

#include <rviz/properties/color_property.h>

#include <rviz/display_group.h>
#include <rviz/visualization_manager.h>
#include <visualization_msgs/InteractiveMarker.h>

using namespace rviz_visual_tools;

std_msgs::ColorRGBA qcolorToColorRGBA(const QColor &qcolor) {
  std_msgs::ColorRGBA color_rgba;
  color_rgba.r = qcolor.redF();   // redF() 返回一个介于0到1之间的浮点数
  color_rgba.g = qcolor.greenF(); // greenF() 返回一个介于0到1之间的浮点数
  color_rgba.b = qcolor.blueF();  // blueF() 返回一个介于0到1之间的浮点数
  color_rgba.a = qcolor.alphaF(); // alphaF() 返回一个介于0到1之间的浮点数
  return color_rgba;
}

void MyRvizVisualTools::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid) {
    mouse_point_ss << " at " << feedback->mouse_point.x << ", " << feedback->mouse_point.y << ", "
                   << feedback->mouse_point.z << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type) {
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    ROS_INFO_STREAM(s.str() << ": button click" << mouse_point_ss.str() << ".");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    ROS_INFO_STREAM(s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    ROS_INFO_STREAM(s.str() << ": pose changed"
                            << "\nposition = " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", "
                            << feedback->pose.position.z << "\norientation = " << feedback->pose.orientation.w << ", "
                            << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y << ", "
                            << feedback->pose.orientation.z << "\nframe: " << feedback->header.frame_id << " time: "
                            << feedback->header.stamp.sec << "sec, " << feedback->header.stamp.nsec << " nsec");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    ROS_INFO_STREAM(s.str() << ": mouse down" << mouse_point_ss.str() << ".");
    break;

  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
    break;
  }

  server_->applyChanges();
}

void MyRvizVisualTools::CreateMenuMarker(visualization_msgs::Marker const &mark)

{
  using namespace visualization_msgs;

  InteractiveMarker int_marker;
  int_marker.header.frame_id = mark.header.frame_id;
  int_marker.pose.position = mark.pose.position;
  int_marker.scale = 1;

  int_marker.name = mark.ns + "/" + std::to_string(mark.id);
  // int_marker.description = "Context Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  control.markers.push_back(mark);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  // server_->setCallback(int_marker.name, &processFeedback);
  menu_handler_.apply(*server_, int_marker.name);

  server_->applyChanges();
}

void MyRvizVisualTools::CreateInteractiveMarkerServer() {
  // create server
  server_.reset(new interactive_markers::InteractiveMarkerServer("MyRvizVisualTools", "", true));

  auto cbk = std::bind(&MyRvizVisualTools::processFeedback, this, std::placeholders::_1);

  menu_handler_.insert("First Entry", cbk);
  menu_handler_.insert("Second Entry", cbk);
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Submenu");
  menu_handler_.insert(sub_menu_handle, "First Entry", cbk);
  menu_handler_.insert(sub_menu_handle, "Second Entry", cbk);
}

MyRvizVisualTools::MyRvizVisualTools(std::string base_frame) : RvizVisualTools(base_frame) {
  this->enableBatchPublishing();

  CreateInteractiveMarkerServer();
}

MyRvizVisualTools::~MyRvizVisualTools() {
  reset();

  // ROS_WARN_STREAM("imark_display_ = " << imark_display_);

  // auto *vm = qobject_cast<rviz::VisualizationManager *>(context_);
  // vm->stopUpdate();

  // vm->getRootDisplayGroup()->takeDisplay(imark_display_);

  // vm->startUpdate();
}

void MyRvizVisualTools::initialize(rviz::Display *parent, rviz::DisplayContext *context) {
  parent_ = parent;
  context_ = context;
  root_node_ = parent_->getSceneNode();

  ns_root_ = new rviz::BoolProperty("ns filter", true, "null", parent_);
  color_root_ = new rviz::ColorProperty("root color", Qt::gray, "new color to all", ns_root_);
  // create_interactive_marker_ = new rviz::BoolProperty("create interactive marker", false, "null", parent_);

  root_node_->getUserObjectBindings().setUserAny(Ogre::Any(parent));

  // imark
  auto *vm = qobject_cast<rviz::VisualizationManager *>(context_);
  auto *imark_display_ = vm->createDisplay("rviz/InteractiveMarkers", "imark", false);
  imark_display_->setTopic("MyRvizVisualTools/update", "");

  vm->getRootDisplayGroup()->takeDisplay(imark_display_);
  parent_->addChild(imark_display_, 1);
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
  ns_root_->removeChildren(1);
  ns_properties_.clear();
}

void MyRvizVisualTools::updateColor(std::string const &ns, QColor const &color) {
  if (ns_properties_.find(ns) == ns_properties_.end()) { return; }

  auto const new_color = qcolorToColorRGBA(color);

  markers_.markers.reserve(ns_filted_node_[ns].size());

  beginInit();

  for (auto *single_node : ns_filted_node_[ns]) {
    if (auto const &mark_view = all_scene_node_[single_node]; mark_view) {
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

void MyRvizVisualTools::CreateNewNS(visualization_msgs::Marker const &mark) {
  // 新的ns,更新筛选器
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

rviz::MarkerBasePtr MyRvizVisualTools::CreateMarkView(visualization_msgs::Marker const &mark) {
  auto *new_scene_node = root_node_->createChildSceneNode();

  auto mark_view = rviz::createMarker2(mark.type, nullptr, context_, new_scene_node);
  mark_view->setMessage(mark);

  ns_filted_node_[mark.ns].push_back(new_scene_node);
  all_scene_node_[new_scene_node] = mark_view;

  if (ns_properties_.find(mark.ns) == ns_properties_.end()) { CreateNewNS(mark); }

  return mark_view;
}

void MyRvizVisualTools::update(float wall_dt, float ros_dt) {
  if (!inited_ || markers_.markers.empty()) { return; }
  for (auto const &mark : markers_.markers) {
    CreateMarkView(mark);
    CreateMenuMarker(mark);
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