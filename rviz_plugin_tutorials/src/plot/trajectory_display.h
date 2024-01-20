#pragma once

// #include <rviz/properties/color_property.h>
// #include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>

// #include <rviz/properties/vector_property.h>
// #include <rviz/properties/enum_property.h>
// #include <rviz/properties/tf_frame_property.h>
#include <rviz/display.h>

#include "display_sync_base.h"
#include "trajectory_widget.h"
#include <deque>
namespace rviz
{
  class IntProperty;
  class EnumProperty;
  class BoolProperty;
  class GroupProperty;
  class ColorProperty;
}

namespace zhito
{
  class TrajectoryPanel;
}

class TrajectoryWidget;
class QCPCurve;

class GraphProperty : public rviz::BoolProperty
{
  Q_OBJECT
public:
  GraphProperty(TrajectoryWidget *plot, Property *parent = nullptr);
  ~GraphProperty();
private Q_SLOTS:
  void UpdateScatterShape();
  void UpdateScatterColor();
  void UpdateScatterSize();
  // void UpdateScatterStyle();
  void UpdateLineStyle();
  void UpdateLineWidth();
  void UpdateLineColor();
  void UpdateEnable();
  void UpdateTopic();
  void SyncInfo();

protected:
  QPen getLinePen() const;
  QCPScatterStyle getScatterStyle() const;

protected:
  static int graph_counts_;
  QTimer dataTimer_; // 检查是否有数据更新
  rviz::EnumProperty *channel_name_prop_ = nullptr;
  // scatter
  rviz::EnumProperty *scatter_type_ = nullptr;
  rviz::ColorProperty *scatter_color_ = nullptr; // scatter color
  rviz::IntProperty *scatter_size_ = nullptr;    // scatter size
  // line
  rviz::EnumProperty *line_type_ = nullptr;   // line type
  rviz::IntProperty *line_width_ = nullptr;   // line width
  rviz::ColorProperty *line_color_ = nullptr; // line color

  QCPCurve *curve_;
  TrajectoryWidget *plot_ = nullptr;
};

class TrajectoryDisplay : public DisplaySyncBase
{
  Q_OBJECT
public:
  TrajectoryDisplay();
  ~TrajectoryDisplay() override;

  // Overrides from Display
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  void onInitialize() override;
  void update(float dt, float ros_dt) override;
private Q_SLOTS:

  void UpdateFocusWhenSelect(); // 将当前视图放置在central widget位置
  void UpdateGraphCount();      //
private:
  TrajectoryWidget *view_ = nullptr;

  rviz::EnumProperty *scatter_type_ = nullptr;
  rviz::EnumProperty *line_type_ = nullptr;
  rviz::IntProperty *counts_prop_ = nullptr;            // 轨迹数目
  rviz::BoolProperty *focus_when_select_ = nullptr;     // 选中时居中
  rviz::EnumProperty *plot_type_prop_ = nullptr;        // 绘图类型
  rviz::BoolProperty *differ_with_main_prop_ = nullptr; // 与主轨迹做差

  std::deque<std::shared_ptr<GraphProperty>> graphs_;
};
