/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <rviz/tool.h>
#include <rviz/display.h>
#include <rviz/properties/bool_property.h>
#include <map>
#include "plot/qcustomplot.h"

namespace Ogre
{
  class SceneNode;
  class Vector3;
}

namespace rviz
{
  class VectorProperty;
  class VisualizationManager;
  class ViewportMouseEvent;
  class EnumProperty;
  class BoolProperty;
}

class DisplaySyncBase;

enum TimeSyncMode
{
  None,
  SyncMain, // 主节点通知其他所有人
  SyncAll   // 所有节点同步
};

class DisplaySyncManager : public rviz::BoolProperty
{
  Q_OBJECT
public:
  DisplaySyncManager();
  DisplaySyncManager(Property *tool_root);
  ~DisplaySyncManager();
  void Initialize(rviz::VisualizationManager *context);

protected:
public:
private Q_SLOTS:
  void onDisplayAdded(rviz::Display *display);
  void onDisplayRemoved(rviz::Display *display);
  void onFocusPointChanged(double const t0, DisplaySyncBase *sender);
  void onFouseRangeChanged(QCPRange const &time_range, DisplaySyncBase *sender);

private:
  rviz::EnumProperty *sync_mode_ = nullptr;
  rviz::Property *tool_root_ = nullptr;
  //! 同一个类型的display名称一致，使用名称做索引时需要注意
  std::list<DisplaySyncBase *> syncers_;
  std::map<DisplaySyncBase *, rviz::BoolProperty *> sync_properties_;
};

namespace rviz_plugin_tutorials
{

  // BEGIN_TUTORIAL
  // Here we declare our new subclass of rviz::Tool.  Every tool
  // which can be added to the tool bar is a subclass of
  // rviz::Tool.
  class TimeSyncTool : public rviz::Tool
  {
    Q_OBJECT
  public:
    TimeSyncTool();
    ~TimeSyncTool();

    virtual void onInitialize();

    virtual void activate();
    virtual void deactivate();

    virtual void load(const rviz::Config &config);
    virtual void save(rviz::Config config) const;

  private:
    DisplaySyncManager *sync_manager_ = nullptr;
    bool is_toggled_on_;
  };
  // END_TUTORIAL

} // end namespace rviz_plugin_tutorials
