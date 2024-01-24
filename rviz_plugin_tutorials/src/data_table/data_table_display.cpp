#include "data_table_display.h"
#include "properties/cached_channel_property.h"
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/int_property.h>

#include "data_table/data_table_widget.h"

DataTableDisplay::DataTableDisplay() {
  InitPersons();

  view_ = new DataTableWidget();
  view_->setDisplaySync(this);

  //
  data_channel_ = new rviz::CachedChannelProperty("data_channel", "", "data_channel", this, SLOT(UpdateChannel()));

  main_interval_ = new rviz::IntProperty("interval", 100, "main grid interval[5,1000]", this, SLOT(UpdateInterval()));
  main_interval_->setMin(5);
  main_interval_->setMax(1000);
  sub_range_ = new rviz::IntProperty("range", 100, "sub grid range [10,100]", this, SLOT(UpdateRange()));
  sub_range_->setMin(10);
  sub_range_->setMax(100);

  connect(&dataTimer_, SIGNAL(timeout()), this, SLOT(SyncInfo()));
}
DataTableDisplay::~DataTableDisplay() {
  if (initialized()) { delete view_; }
}

void DataTableDisplay::SyncInfo() {
  // 去buffer里面查询通道更新
  if (!view_ || data_channel_->getStdString() == "") { return; }
  // 当前时间
  double t0 = view_->getLastDataTime();
  auto msgs = g_cacher_->GetProtoWithChannleName(data_channel_->getStdString(), t0);
  if (!msgs.empty()) {
    view_->UpdateData(msgs);
    qDebug() << QString("add size =  %1").arg(msgs.size());
  }

  // 去buffer里面查询数据更新
}

void DataTableDisplay::UpdateChannel() {
  dataTimer_.stop();

  auto channel_name = data_channel_->getStdString();
  auto type_name = g_cacher_->GetTypeNameWithChannelName(channel_name);
  view_->setChannelName(data_channel_->getString());
  view_->setDataTypeName(type_name);

  dataTimer_.start(500);
}

// Overrides from Display
void DataTableDisplay::onInitialize() { setAssociatedWidget(view_); }

void DataTableDisplay::UpdateInterval() { view_->setMainInterval(main_interval_->getInt()); }
void DataTableDisplay::UpdateRange() { view_->setSubRange(sub_range_->getInt()); }

void DataTableDisplay::update(float dt, float ros_dt) {}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(DataTableDisplay, rviz::Display)