#pragma once

#include "plot/qcustomplot.h"
#include <rviz/display.h>
#include <map>

class ITimeSync;

// namespace rviz
// {
//   class EnumProperty;
//   class BoolProperty;
//   class GroupProperty;
//   class DisplayContext;
// }

// class DisplayViewManager
// {
// public:
//   DisplaySyncBase *sync_display_ = nullptr;
//   ITimeSync *view_ = nullptr;
// };

class DisplaySyncBase : public rviz::Display
{
  Q_OBJECT
public:
  DisplaySyncBase();
  virtual ~DisplaySyncBase();
  virtual ITimeSync *getView() = 0;
Q_SIGNALS:
  void FocusPointChanged(double const t0);
  void FouseRangeChanged(QCPRange const &time_range);
public Q_SLOTS:
  void onFocusPoint(double const t0, bool update_view = true, bool emit_signal = true);
  void onFouseRange(QCPRange const &time_range, bool update_view = true, bool emit_signal = true);

protected:
public:
protected:
  QCPRange selected_range_ = {0, 0}; // 当前感兴趣的范围
  double selected_t0_s_ = 0;         // 选中的时刻
};
