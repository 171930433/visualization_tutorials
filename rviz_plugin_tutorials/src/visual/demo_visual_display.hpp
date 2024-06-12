#pragma once

#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/display_context.h>
#include <rviz/visualization_manager.h>


#include "visual/shape_visual.hpp"
#include "visual/bline_visual.hpp"
#include "visual/my_pointcloud.hpp"



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
};