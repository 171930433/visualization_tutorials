#pragma once

#include <rviz_common/properties/int_property.hpp>

#include <rviz_common/display.hpp>

#include "display_sync_base.h"

namespace rviz_common {
namespace properties {
class IntProperty;
class EditableEnumProperty;
class CachedChannelProperty;
} // namespace properties

} // namespace rviz_common

class DataTableWidget;

class DataTableDisplay : public DisplaySyncBase {
  Q_OBJECT
public:
  DataTableDisplay();
  ~DataTableDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float dt, float ros_dt) override;

private Q_SLOTS:
  void UpdateInterval();
  void UpdateRange();
  void UpdateChannel();
  void SyncInfo();

private:
  DataTableWidget *view_ = nullptr;

  rviz_common::properties::IntProperty *main_interval_;
  rviz_common::properties::IntProperty *sub_range_;
  rviz_common::properties::CachedChannelProperty *data_channel_; // 错误，需要使用cyber_topic_property
  QTimer dataTimer_;                          // 检查是否有数据更新
};
