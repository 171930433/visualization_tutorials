#include "display_sync_base.h"
#include "time_sync.h"

// #include <rviz/properties/enum_property.h>
// #include <rviz/properties/bool_property.h>

// #include <rviz/display_context.h>
// #include <rviz/visualization_manager.h>
// #include <rviz/display_group.h>

DisplaySyncBase::DisplaySyncBase() : Display()
{
}

DisplaySyncBase::~DisplaySyncBase()
{
}

ITimeSync *DisplaySyncBase::getView()
{
  return dynamic_cast<ITimeSync *>(this->getAssociatedWidget());
};

void DisplaySyncBase::onFocusPoint(double const t0, bool update_view, bool emit_signal)
{
  // qDebug() << this->getName() << "DisplaySyncBase::onFocusPoint" << QString("%1").arg(t0, 0, 'f', 3) << QString("update_view=%1 emit_signal=%2").arg(update_view).arg(emit_signal);
  // qDebug() << QString("t0=%1 selected_t0_s_=%2 thesame=%3 == %4").arg(t0, 0, 'f', 3).arg(selected_t0_s_, 0, 'f', 3).arg(std::abs(t0 - selected_t0_s_) >= 1e-6).arg(t0 != selected_t0_s_);
  if (t0 != selected_t0_s_)
  {
    selected_t0_s_ = t0;
    if (update_view)
    {
      getView()->FocusPoint(selected_t0_s_);
    }
    if (emit_signal)
    {
      Q_EMIT FocusPointChanged(t0);
    }
  }
}
void DisplaySyncBase::onFouseRange(QCPRange const &time_range, bool update_view, bool emit_signal)
{
  if (time_range != selected_range_)
  {
    selected_range_ = time_range;
    if (update_view)
    {
      getView()->FouseRange(selected_range_);
    }
    if (emit_signal)
    {
      Q_EMIT FouseRangeChanged(selected_range_);
    }
  }
}