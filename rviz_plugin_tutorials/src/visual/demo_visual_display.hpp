#pragma once

#include <rviz/default_plugin/marker_array_display.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>

#include "visual/bline_visual.hpp"
#include "visual/my_pointcloud.hpp"
#include "visual/shape_visual.hpp"


//
#include "visual/my_rviz_visual_tools.hpp"





class DemoVisualDisplay : public rviz::Display {
public:
  DemoVisualDisplay();

  void onInitialize() override;

protected:
  void update(float wall_dt, float ros_dt) override;

protected:
  ShapeVisual shape_;
  MyLine bline_;
  MyPointCloud pc_;
  // 最佳自定义图元绘制
protected:
protected:
  std::shared_ptr<MyRvizVisualTools> rvt_ = nullptr;

};