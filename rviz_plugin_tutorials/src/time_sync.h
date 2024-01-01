#pragma once

#include "plot/qcustomplot.h"
#include <rviz/display.h>
#include <map>

namespace rviz
{
  class EnumProperty;
  class BoolProperty;
  class GroupProperty;
  class DisplayContext;
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

class DisplaySyncBase
{
public:
  virtual ~DisplaySyncBase();

protected:
  DisplaySyncBase(rviz::Display *display, ITimeSync *sync);
  void onInitialize(rviz::DisplayContext *context);

public:
  virtual void FocusPoint(double const t0);
  virtual void FouseRange(QCPRange const &time_range);
  ITimeSync *view_ = nullptr;
  rviz::Display *display_ = nullptr;
};

class DisplaySyncManager : public rviz::Display
{
  Q_OBJECT
public:
  DisplaySyncManager();
  ~DisplaySyncManager();

protected:
public:
  void AddSyncer(rviz::Display *display, DisplaySyncBase *sync_base);
  void RemoveSyncer(rviz::Display *display);
private Q_SLOTS:

private:
  rviz::EnumProperty *sync_mode_ = nullptr;

  std::map<rviz::Display *, DisplaySyncBase *> g_syncers_;
};

extern DisplaySyncManager* g_sync_manager;
