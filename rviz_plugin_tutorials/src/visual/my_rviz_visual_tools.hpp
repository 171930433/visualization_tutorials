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