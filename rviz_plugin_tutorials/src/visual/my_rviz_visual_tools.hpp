#pragma once

#include "rviz_visual_tools/rviz_visual_tools.h"

#include <rviz/default_plugin/marker_utils.h>
#include <rviz/default_plugin/markers/marker_base.h>
#include <rviz/display.h>

#include <OgreSceneNode.h>
#include <unordered_map>

class MyRvizVisualTools : public rviz_visual_tools::RvizVisualTools, public QObject {
public:
  using rviz_visual_tools::RvizVisualTools::RvizVisualTools;
  void initialize(rviz::Display *parent, rviz::DisplayContext *context);

  void update();

public:
  bool publishPlaneRect(Eigen::Vector3d const &pos,
                        Eigen::Vector3d const &normal,
                        rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                        double x_width = 1.0,
                        double y_width = 1.0,
                        std::string const ns = "PlaneRect");

  bool publishPlaneCircle(Eigen::Vector3d const &pos,
                          Eigen::Vector3d const &normal,
                          double const radius_m,
                          rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                          std::string const ns = "PlaneCircle");

  bool publishPlaneTriangle(Eigen::Vector3d const &pos,
                            Eigen::Vector3d const &normal,
                            double const length_m,
                            rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                            std::string const ns = "PlaneTriangle");

  // 绘制正多边形
  bool publishPlaneRegularPolygon(Eigen::Vector3d const &pos,
                            Eigen::Vector3d const &normal,
                            int const line_count,   // 多边形的边数
                            double const length_m,  // 多边形的边长
                            rviz_visual_tools::colors color = rviz_visual_tools::TRANSLUCENT,
                            std::string const ns = "PlaneRegularPolygon");

protected:
  rviz::MarkerBase *CreateMarkView(visualization_msgs::Marker const &mark);

protected:
  rviz::Display *parent_;
  rviz::DisplayContext *context_;
  Ogre::SceneNode *scene_node_;
  std::unordered_map<rviz::MarkerBase *, Ogre::SceneNode *> all_scene_node_; // 获取每一个mark对应的scene_node
  std::unordered_map<std::string, std::list<Ogre::SceneNode *>> ns_filted_node_; // 根据ns分组得到的node
  std::unordered_map<std::string, rviz::BoolProperty *> ns_properties_;
  rviz::BoolProperty *ns_root_;
};