#pragma once

#include "plot/qcustomplot.h"
#include <rviz/display.h>
#include <map>

class DisplaySyncBase;

class ITimeSync
{
public:
  void onFocusPoint(double const t0, bool update_view = true, bool emit_signal = true);
  void onFouseRange(QCPRange const &time_range, bool update_view = true, bool emit_signal = true);

  // 每个view类需要实际实现的锁定函数
  virtual void FocusPoint(double const t0) = 0;
  virtual void FouseRange(QCPRange const &time_range) = 0;

protected:
  virtual DisplaySyncBase *getDisplaySync() = 0;

protected:
};
