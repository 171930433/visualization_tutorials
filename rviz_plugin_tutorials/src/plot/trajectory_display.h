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
  void UpdateLineStyle();
  void UpdateEnable();
  void UpdateTopic();
  void SyncInfo();

protected:

protected:
  static int graph_counts_;
  QTimer dataTimer_; // 检查是否有数据更新
  rviz::EnumProperty *channel_name_prop_ = nullptr;
  rviz::EnumProperty *scatter_type_ = nullptr;
  rviz::EnumProperty *line_type_ = nullptr;
  QCPCurve *curve_;
  TrajectoryWidget *plot_ = nullptr;
};

class TrajectoryDisplay : public DisplaySyncBase
{
  Q_OBJECT
public:
  TrajectoryDisplay();
  ~TrajectoryDisplay() override;

  // 需要在 Initialize 之前调用,以确定在onInitialize 绑定有效
  void setView(TrajectoryWidget *view) { view_ = view; }
  ITimeSync *getView() override;
  void setPanel(zhito::TrajectoryPanel *panel) { panel_ = panel; }

  // Overrides from Display
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  void onInitialize() override;
  void update(float dt, float ros_dt) override;
private Q_SLOTS:
  void UpdateScatterShape();
  void UpdateLineStyle();
  void Swap2Central();          // 将当前视图放置在central widget位置
  void UpdateFocusWhenSelect(); // 将当前视图放置在central widget位置
  void UpdateGraphCount();      //

private:
  TrajectoryWidget *view_ = nullptr;
  zhito::TrajectoryPanel *panel_ = nullptr;

  rviz::EnumProperty *scatter_type_ = nullptr;
  rviz::EnumProperty *line_type_ = nullptr;
  rviz::BoolProperty *swap2central_ = nullptr;
  rviz::IntProperty *counts_prop_ = nullptr;        // 轨迹数目
  rviz::BoolProperty *focus_when_select_ = nullptr; // 选中时居中

  // std::list<GraphProperty *> graphs_;
  // std::list<std::shared_ptr<GraphProperty>> graphs_;
  std::deque<std::shared_ptr<GraphProperty>> graphs_;
};
