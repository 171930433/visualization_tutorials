#pragma once

// #include <rviz/properties/color_property.h>
// #include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>

// #include <rviz/properties/vector_property.h>
// #include <rviz/properties/enum_property.h>
// #include <rviz/properties/tf_frame_property.h>
#include <rviz/display.h>

#include "time_sync.h"

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
  GraphProperty(QCPCurve *graph, Property *parent = nullptr);

private Q_SLOTS:
  void UpdateScatterShape();
  void UpdateLineStyle();
  void UpdateEnable();

protected:
  rviz::EnumProperty *scatter_type_ = nullptr;
  rviz::EnumProperty *line_type_ = nullptr;
  QCPCurve *graph_;
};

class TrajectoryDisplay : public rviz::Display, public DisplaySyncBase
{
  Q_OBJECT
public:
  TrajectoryDisplay();
  ~TrajectoryDisplay() override;

  // 需要在 Initialize 之前调用,以确定在onInitialize 绑定有效
  void setView(TrajectoryWidget *view) { view_ = view; }
  void setPanel(zhito::TrajectoryPanel *panel) { panel_ = panel; }

  // Overrides from Display
  void onInitialize() override;
  void update(float dt, float ros_dt) override;

private Q_SLOTS:
  void UpdateScatterShape();
  void UpdateLineStyle();
  void Swap2Central(); // 将当前视图放置在central widget位置

private:
  TrajectoryWidget *view_ = nullptr;
  zhito::TrajectoryPanel *panel_ = nullptr;

  rviz::EnumProperty *scatter_type_ = nullptr;
  rviz::EnumProperty *line_type_ = nullptr;
  rviz::BoolProperty *swap2central_ = nullptr;

  std::map<std::string, GraphProperty *> graphs_;
};
