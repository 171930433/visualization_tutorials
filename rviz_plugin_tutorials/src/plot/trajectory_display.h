#pragma once

#include "properties/sub_plot_property.h"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>

#include <rviz_common/display.hpp>

#include "display_sync_base.h"
#include "trajectory_widget.h"
#include <deque>

namespace rviz_common {
namespace properties {

class IntProperty;
class EnumProperty;
class BoolProperty;
class GroupProperty;
class ColorProperty;

} // namespace properties
} // namespace rviz_common

class TrajectoryWidget;
class QCPCurve;

class TrajectoryDisplay : public DisplaySyncBase {
  Q_OBJECT
public:
  TrajectoryDisplay();
  ~TrajectoryDisplay() override;

  // Overrides from Display
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;
  void onInitialize() override;
  void update(float dt, float ros_dt) override;
private Q_SLOTS:

  void UpdateFocusWhenSelect(); // 将当前视图放置在central widget位置
  void UpdateGraphCount();      //
  void UpdateTopic();
  void SyncInfo();

private:
  TrajectoryWidget *view_ = nullptr;
  QTimer dataTimer_; // 检查是否有数据更新

  rviz_common::properties::IntProperty *counts_prop_ = nullptr;        // 轨迹数目
  rviz_common::properties::BoolProperty *focus_when_select_ = nullptr; // 选中时居中

  rviz_common::properties::VectorXSubCurve graphs_;
};
