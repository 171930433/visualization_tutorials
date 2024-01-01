#include "display_sync_base.h"
#include "time_sync.h"

#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>

#include <rviz/display_context.h>
#include <rviz/visualization_manager.h>
#include <rviz/display_group.h>

DisplaySyncBase::DisplaySyncBase() : Display()
{
}

DisplaySyncBase::~DisplaySyncBase()
{
}

void DisplaySyncBase::onFocusPoint(double const t0, bool update_view)
{
  qDebug() << this->getName() << "DisplaySyncBase::onFocusPoint" << QString("%1").arg(t0, 0, 'f', 3);

  if (t0 != selected_t0_s_)
  {
    selected_t0_s_ = t0;
    if (update_view)
    {
      getView()->FocusPoint(selected_t0_s_);
    }
    Q_EMIT FocusPointChanged(t0);
  }
}
void DisplaySyncBase::onFouseRange(QCPRange const &time_range, bool update_view)
{
  if (time_range != selected_range_)
  {
    selected_range_ = time_range;
    if (update_view)
    {
      getView()->FouseRange(selected_range_);
    }
    Q_EMIT FouseRangeChanged(selected_range_);
  }
}

//

DisplaySyncManager::DisplaySyncManager()
{
  sync_mode_ = new rviz::EnumProperty("sync mode", "One2X", "the point type of trajectory", this);
  sync_mode_->addOption("None", TimeSyncMode::None);
  sync_mode_->addOption("SyncMain", TimeSyncMode::SyncMain);
  sync_mode_->addOption("SyncAll", TimeSyncMode::SyncAll);
}

DisplaySyncManager::~DisplaySyncManager()
{
  // syncers_.clear();
  // sync_properties_.clear();
}

void DisplaySyncManager::onInitialize()
{
  // 载入当前已经加载的时间同步型display
  auto vm = qobject_cast<rviz::VisualizationManager *>(context_);
  auto dg = vm->getRootDisplayGroup();
  int const count = dg->numDisplays();
  for (int i = 0; i < count; i++)
  {
    onDisplayAdded(dg->getDisplayAt(i));
  }
  // 绑定增加与删除消息
  connect(dg, SIGNAL(displayAdded(rviz::Display *)), this, SLOT(onDisplayAdded(rviz::Display *)));
  connect(dg, SIGNAL(displayRemoved(rviz::Display *)), this, SLOT(onDisplayRemoved(rviz::Display *)));
}

void DisplaySyncManager::onDisplayAdded(rviz::Display *display)
{
  DisplaySyncBase *sync_one = qobject_cast<DisplaySyncBase *>(display);
  if (!sync_one)
  {
    return;
  }

  syncers_[sync_one->getName().toStdString()] = sync_one;
  sync_properties_[sync_one->getName().toStdString()] = new rviz::BoolProperty(sync_one->getName(), true, "sync options", this);
  // 消息绑定
  connect(sync_one, SIGNAL(FocusPointChanged(double const)), this, SLOT(onFocusPointChanged(double const)));
  connect(sync_one, SIGNAL(FouseRangeChanged(QCPRange const &)), this, SLOT(onFouseRangeChanged(QCPRange const &)));

  qDebug() << " AddSyncer " << sync_one->getName();
}

void DisplaySyncManager::onFocusPointChanged(double const t0)
{
  qDebug() << "DisplaySyncManager::onFocusPointChanged" << QString("%1").arg(t0, 0, 'f', 3);

  for (auto [key, sync] : syncers_)
  {
    sync->onFocusPoint(t0);
  }
}
void DisplaySyncManager::onFouseRangeChanged(QCPRange const &time_range)
{
  for (auto [key, sync] : syncers_)
  {
    sync->onFouseRange(time_range);
  }
}

void DisplaySyncManager::onDisplayRemoved(rviz::Display *display)
{
  DisplaySyncBase *sync_one = qobject_cast<DisplaySyncBase *>(display);
  if (!sync_one)
  {
    return;
  }

  syncers_.erase(sync_one->getName().toStdString());
  sync_properties_.erase(sync_one->getName().toStdString());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(DisplaySyncManager, rviz::Display)