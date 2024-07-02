#pragma once

#include "rviz_visual_tools/rviz_visual_tools.h"

#include <rviz/default_plugin/marker_utils.h>
#include <rviz/default_plugin/markers/marker_base.h>
#include <rviz/display.h>

#include <rviz/properties/color_property.h>

#include <OgreSceneNode.h>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index_container.hpp>
#include <unordered_map>

struct MarkIndex {
  struct Mark {};
  struct Node {};
  struct Ns {};
  struct Id {};

  rviz::MarkerBase *mark_ = nullptr;
  Ogre::SceneNode *scene_node_ = nullptr;
  rviz::BoolProperty *ns_filted_ = nullptr;
  const rviz::MarkerID getID() const { return mark_->getID(); }
  //
  rviz::BoolProperty *root_ns_filted_ = nullptr;
  Ogre::SceneNode *root_node_ = nullptr;
  ~MarkIndex() {
    if (mark_) {
      delete mark_;
      mark_ = nullptr;
    }
    if (scene_node_) {
      root_node_->removeAndDestroyChild(scene_node_->getName());
      scene_node_ = nullptr;
    }
  }
};

using namespace boost::multi_index;

class MyRvizVisualTools : public rviz_visual_tools::RvizVisualTools, public QObject {
public:
  // using rviz_visual_tools::RvizVisualTools::RvizVisualTools;
  explicit MyRvizVisualTools(std::string base_frame);
  ~MyRvizVisualTools();

  void initialize(rviz::Display *parent, rviz::DisplayContext *context);

  void update();

  void beginInit() { inited_ = false; } // 开始添加元素时调用
  void endInit() { inited_ = true; }    // 结束添加元素时调用

  visualization_msgs::MarkerArray &markers() { return markers_; } // 只可在begin与end之间访问

  void reset();

protected:
  MarkIndex CreateMarkIndex(rviz::MarkerBase *mark_, Ogre::SceneNode *scene_node_, rviz::BoolProperty *ns_filted_) {
    return MarkIndex{mark_, scene_node_, ns_filted_, ns_root_, root_node_};
  }

public:
  bool publishRect(Eigen::Vector3d const &pos,
                   Eigen::Vector3d const &normal,
                   rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                   double x_width = 1.0,
                   double y_width = 1.0,
                   std::string const ns = "Rect");

  bool publishCircle(Eigen::Vector3d const &pos,
                     Eigen::Vector3d const &normal,
                     double const radius_m,
                     rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                     std::string const ns = "Circle");

  bool publishRegularTriangle(Eigen::Vector3d const &pos,
                              Eigen::Vector3d const &normal,
                              double const length_m,
                              rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                              std::string const ns = "Triangle");

  // 绘制正多边形 固定边长
  bool publishRegularPolygon(Eigen::Vector3d const &pos,
                             Eigen::Vector3d const &normal,
                             int const line_count,  // 多边形的边数
                             double const length_m, // 多边形的边长
                             rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                             std::string const ns = "RegularPolygon");
  // 绘制正多边形 固定外接圆半径
  bool publishRegularPolygon2(Eigen::Vector3d const &pos,
                              Eigen::Vector3d const &normal,
                              int const line_count,  // 多边形的边数
                              double const radius_m, // 多边形的边长
                              rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                              std::string const ns = "RegularPolygon");

protected:
  rviz::MarkerBasePtr CreateMarkView(visualization_msgs::Marker const &mark);
  void CreateNewNS(visualization_msgs::Marker const &mark);
  void reset_ns(std::string const &ns);
  void updateColor(std::string const &ns, QColor const &new_color);

protected:
  rviz::Display *parent_;
  rviz::DisplayContext *context_;
  Ogre::SceneNode *root_node_;
  rviz::BoolProperty *ns_root_;
  rviz::ColorProperty *color_root_;
  std::unordered_map<Ogre::SceneNode *, rviz::MarkerBasePtr> all_scene_node_; // 获取每一个mark对应的scene_node
  std::unordered_map<std::string, std::list<Ogre::SceneNode *>> ns_filted_node_; // 根据ns分组得到的node
  std::unordered_map<std::string, rviz::BoolProperty *> ns_properties_;
  //
  std::atomic_bool inited_ = {false};
  //
};