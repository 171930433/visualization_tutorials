#pragma once

#include "plot/qcustomplot.h"
#include <rviz/display.h>
#include <map>

class ITimeSync;

namespace rviz
{
  class EnumProperty;
  class BoolProperty;
  class GroupProperty;
  class DisplayContext;
}

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

enum TimeSyncMode
{
  None,
  SyncMain, // 主节点通知其他所有人
  SyncAll   // 所有节点同步
};

class DisplaySyncManager : public rviz::Display
{
  Q_OBJECT
public:
  DisplaySyncManager();
  ~DisplaySyncManager();
  void onInitialize() override;

protected:
public:
private Q_SLOTS:
  void onDisplayAdded(rviz::Display *display);
  void onDisplayRemoved(rviz::Display *display);
  void onFocusPointChanged(double const t0);
  void onFouseRangeChanged(QCPRange const &time_range);

private:
  rviz::EnumProperty *sync_mode_ = nullptr;
  std::map<std::string, rviz::BoolProperty *> sync_properties_;

  //! 同一个类型的display名称一致，使用名称做索引时需要注意
  std::map<std::string, DisplaySyncBase *> syncers_;
};