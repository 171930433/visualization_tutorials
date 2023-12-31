#pragma once

#include "plot/qcustomplot.h"
#include <rviz/display.h>
#include <map>

namespace rviz
{
  class EnumProperty;
  class BoolProperty;
  class GroupProperty;
}

enum TimeSyncMode
{
  One2X,  // 主节点通知其他所有人
  SyncAll // 所有节点同步
};

class ITimeSync
{
public:
  virtual void FocusPoint(double const t0) = 0;
  virtual void FouseRange(QCPRange const &time_range) = 0;
};

class TimeSyncDisplay : public rviz::Display
{
  Q_OBJECT
public:
  TimeSyncDisplay();
  ~TimeSyncDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float dt, float ros_dt) override;

private Q_SLOTS:

private:
  rviz::EnumProperty *sync_mode_ = nullptr;
  std::map<std::string, ITimeSync *> syncers_;
};
