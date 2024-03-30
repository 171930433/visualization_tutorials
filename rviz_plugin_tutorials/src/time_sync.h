#pragma once

#include "plot/qcustomplot.h"
#include <rviz_common/display.hpp>
#include <map>

class DisplaySyncBase;

class ITimeSync
{
public:
  void onFocusPoint(double const t0, bool update_view = true, bool emit_signal = true);
  void onFouseRange(QCPRange const &time_range, bool update_view = true, bool emit_signal = true);

  // 每个view类需要实际实现的锁定函数, 对于view来说，需要区分出来是同步信号还是人为编辑
  virtual void FocusPoint(double const t0) = 0;
  virtual void FouseRange(QCPRange const &time_range) = 0;

  // get set display
  void setDisplaySync(DisplaySyncBase *sync_display) { sync_display_ = sync_display; }
  DisplaySyncBase *getDisplaySync() { return sync_display_; }
protected:

protected:
  DisplaySyncBase *sync_display_ = nullptr;
};
