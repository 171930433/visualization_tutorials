#pragma once

// #include <rviz/properties/color_property.h>
// #include <rviz/properties/float_property.h>
// #include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>

// #include <rviz/properties/vector_property.h>
// #include <rviz/properties/enum_property.h>
// #include <rviz/properties/tf_frame_property.h>
#include <rviz/display.h>

#include "display_sync_base.h"

namespace rviz
{
  class IntProperty;
  class EditableEnumProperty;
}

class DataTableWidget;

class DataTableDisplay : public DisplaySyncBase
{
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
  void ListCurrentChannel();
  void SyncInfo();

private:
  DataTableWidget *view_ = nullptr;

  rviz::IntProperty *main_interval_;
  rviz::IntProperty *sub_range_;
  rviz::EditableEnumProperty *data_channel_; // 错误，需要使用cyber_topic_property
  QTimer dataTimer_; // 检查是否有数据更新

};
