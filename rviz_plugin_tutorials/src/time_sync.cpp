#include "time_sync.h"

#include <rviz/properties/enum_property.h>

TimeSyncDisplay::TimeSyncDisplay()
{
  sync_mode_ = new rviz::EnumProperty("sync mode", "One2X", "the point type of trajectory", this);
  sync_mode_->addOption("One2X", TimeSyncMode::One2X);
  sync_mode_->addOption("SyncAll", TimeSyncMode::SyncAll);
}

TimeSyncDisplay::~TimeSyncDisplay()
{
}

void TimeSyncDisplay::onInitialize()
{
}
void TimeSyncDisplay::update(float dt, float ros_dt)
{
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TimeSyncDisplay, rviz::Display)  