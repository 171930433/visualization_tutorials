/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>

#include <rviz/display_context.h>
#include <rviz/display_group.h>

#include "time_sync_tool.h"
#include "display_sync_base.h"

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

  syncers_.push_back(sync_one);
  sync_properties_[sync_one] = new rviz::BoolProperty(sync_one->getName(), true, "sync options", this);
  // 消息绑定
  connect(sync_one, SIGNAL(FocusPointChanged(double const)), this, SLOT(onFocusPointChanged(double const)));
  connect(sync_one, SIGNAL(FouseRangeChanged(QCPRange const &)), this, SLOT(onFouseRangeChanged(QCPRange const &)));

  qDebug() << " AddSyncer " << sync_one->getName();
}

void DisplaySyncManager::onFocusPointChanged(double const t0)
{
  // qDebug() << "begin----------------DisplaySyncManager::onFocusPointChanged" << QString("%1").arg(t0, 0, 'f', 3);

  for (auto sync : syncers_)
  {
    if (sync_properties_[sync]->getBool())
    {
      sync->onFocusPoint(t0, true, false);
    }
  }
  // qDebug() << "end-------------------DisplaySyncManager::onFocusPointChanged" << QString("%1").arg(t0, 0, 'f', 3);
}
void DisplaySyncManager::onFouseRangeChanged(QCPRange const &time_range)
{
  for (auto sync : syncers_)
  {
    if (sync_properties_[sync]->getBool())
    {
      sync->onFouseRange(time_range, true, false);
    }
  }
}

void DisplaySyncManager::onDisplayRemoved(rviz::Display *display)
{
  DisplaySyncBase *sync_one = qobject_cast<DisplaySyncBase *>(display);
  if (!sync_one)
  {
    return;
  }

  syncers_.remove(sync_one);
  // 删除当前display
  this->takeChild(sync_properties_[sync_one]);
  sync_properties_.erase(sync_one);
}

namespace rviz_plugin_tutorials
{

  // BEGIN_TUTORIAL
  // Construction and destruction
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // The constructor must have no arguments, so we can't give the
  // constructor the parameters it needs to fully initialize.
  //
  // Here we set the "shortcut_key_" member variable defined in the
  // superclass to declare which key will activate the tool.
  TimeSyncTool::TimeSyncTool()
  {
    sync_manager_ = new DisplaySyncManager();
    sync_manager_->setName(QString("time_sync_manager"));
  }

  // The destructor destroys the Ogre scene nodes for the flags so they
  // disappear from the 3D scene.  The destructor for a Tool subclass is
  // only called when the tool is removed from the toolbar with the "-"
  // button.
  TimeSyncTool::~TimeSyncTool()
  {
    context_->getRootDisplayGroup()->takeDisplay(sync_manager_);
  }

  // onInitialize() is called by the superclass after scene_manager_ and
  // context_ are set.  It should be called only once per instantiation.
  // This is where most one-time initialization work should be done.
  // onInitialize() is called during initial instantiation of the tool
  // object.  At this point the tool has not been activated yet, so any
  // scene objects created should be invisible or disconnected from the
  // scene at this point.
  //
  // In this case we load a mesh object with the shape and appearance of
  // the flag, create an Ogre::SceneNode for the moving flag, and then
  // set it invisible.
  void TimeSyncTool::onInitialize()
  {
    qobject_cast<rviz::VisualizationManager *>(context_)->addDisplay(sync_manager_, true);
  }

  // Activation and deactivation
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // activate() is called when the tool is started by the user, either
  // by clicking on its button in the toolbar or by pressing its hotkey.
  //
  // First we set the moving flag node to be visible, then we create an
  // rviz::VectorProperty to show the user the position of the flag.
  // Unlike rviz::Display, rviz::Tool is not a subclass of
  // rviz::Property, so when we want to add a tool property we need to
  // get the parent container with getPropertyContainer() and add it to
  // that.
  //
  // We wouldn't have to set current_flag_property_ to be read-only, but
  // if it were writable the flag should really change position when the
  // user edits the property.  This is a fine idea, and is possible, but
  // is left as an exercise for the reader.
  void TimeSyncTool::activate()
  {
  }

  // deactivate() is called when the tool is being turned off because
  // another tool has been chosen.
  //
  // We make the moving flag invisible, then delete the current flag
  // property.  Deleting a property also removes it from its parent
  // property, so that doesn't need to be done in a separate step.  If
  // we didn't delete it here, it would stay in the list of flags when
  // we switch to another tool.
  void TimeSyncTool::deactivate()
  {
  }

  // Handling mouse events
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // processMouseEvent() is sort of the main function of a Tool, because
  // mouse interactions are the point of Tools.
  //
  // We use the utility function rviz::getPointOnPlaneFromWindowXY() to
  // see where on the ground plane the user's mouse is pointing, then
  // move the moving flag to that point and update the VectorProperty.
  //
  // If this mouse event was a left button press, we want to save the
  // current flag location.  Therefore we make a new flag at the same
  // place and drop the pointer to the VectorProperty.  Dropping the
  // pointer means when the tool is deactivated the VectorProperty won't
  // be deleted, which is what we want.

  // Loading and saving the flags
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Tools with a fixed set of Property objects representing adjustable
  // parameters are typically just created in the tool's constructor and
  // added to the Property container (getPropertyContainer()).  In that
  // case, the Tool subclass does not need to override load() and save()
  // because the default behavior is to read all the Properties in the
  // container from the Config object.
  //
  // Here however, we have a list of named flag positions of unknown
  // length, so we need to implement save() and load() ourselves.
  //
  // We first save the class ID to the config object so the
  // rviz::ToolManager will know what to instantiate when the config
  // file is read back in.
  void TimeSyncTool::save(rviz::Config config) const
  {
  }

  // In a tool's load() function, we don't need to read its class
  // because that has already been read and used to instantiate the
  // object before this can have been called.
  void TimeSyncTool::load(const rviz::Config &config)
  {
  }

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::TimeSyncTool, rviz::Tool)
// END_TUTORIAL

// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(DisplaySyncManager, rviz::Display)