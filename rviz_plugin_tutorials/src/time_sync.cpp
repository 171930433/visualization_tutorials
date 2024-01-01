#include "time_sync.h"

#include <rviz/properties/enum_property.h>

#include <rviz/display_context.h>
#include <rviz/visualization_manager.h>

DisplaySyncManager *g_sync_manager = nullptr;

DisplaySyncBase::DisplaySyncBase(rviz::Display *display, ITimeSync *sync) : display_(display), view_(sync)
{
}
void DisplaySyncBase::onInitialize(rviz::DisplayContext *context)
{
  if (!g_sync_manager)
  {
    g_sync_manager = new DisplaySyncManager();
    g_sync_manager->setName(QString("Syncer"));
    // 给view
    qobject_cast<rviz::VisualizationManager *>(context)->addDisplay(g_sync_manager, true);
  }

  g_sync_manager->AddSyncer(display_, this);
}

DisplaySyncBase::~DisplaySyncBase()
{
  g_sync_manager->RemoveSyncer(display_);
}

void DisplaySyncBase::FocusPoint(double const t0)
{
  if (view_)
  {
    view_->FocusPoint(t0);
  }
}
void DisplaySyncBase::FouseRange(QCPRange const &time_range)
{
  if (view_)
  {
    view_->FouseRange(time_range);
  }
}

void DisplaySyncManager::AddSyncer(rviz::Display *display, DisplaySyncBase *sync_base)
{
  g_syncers_[display] = sync_base;
}
void DisplaySyncManager::RemoveSyncer(rviz::Display *display)
{
  g_syncers_.erase(display);
}

DisplaySyncManager::DisplaySyncManager()
{
  sync_mode_ = new rviz::EnumProperty("sync mode", "One2X", "the point type of trajectory", this);
  sync_mode_->addOption("One2X", TimeSyncMode::One2X);
  sync_mode_->addOption("SyncAll", TimeSyncMode::SyncAll);
}

DisplaySyncManager::~DisplaySyncManager()
{
}

// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(DisplaySyncManager, rviz::Display)