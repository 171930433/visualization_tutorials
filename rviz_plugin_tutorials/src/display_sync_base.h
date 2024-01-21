#pragma once

#include "plot/qcustomplot.h"
#include <rviz/display.h>
#include <map>

class ITimeSync;

class DisplaySyncBase : public rviz::Display
{
  Q_OBJECT
public:
  DisplaySyncBase();
  virtual ~DisplaySyncBase();
  virtual ITimeSync *getView();
  rviz::DisplayContext *getContext() { return context_; }

Q_SIGNALS:
  void FocusPointChanged(double const t0, DisplaySyncBase *sender);
  void FouseRangeChanged(QCPRange const &time_range, DisplaySyncBase *sender);
public Q_SLOTS:
  void onFocusPoint(double const t0, bool update_view = true, bool emit_signal = true);
  void onFouseRange(QCPRange const &time_range, bool update_view = true, bool emit_signal = true);

protected:
public:
protected:
  QCPRange selected_range_ = {0, 0}; // 当前感兴趣的范围
  double selected_t0_s_ = 0;         // 选中的时刻
};
