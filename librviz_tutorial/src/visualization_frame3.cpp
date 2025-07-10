/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#include "visualization_frame3.hpp"

#include <DockAreaWidget.h>
#include <DockManager.h>
#include <OgreMaterialManager.h>
#include <OgreMeshManager.h>
#include <OgreRenderWindow.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <QApplication>     // NOLINT cpplint cannot handle include order here
#include <QCloseEvent>      // NOLINT cpplint cannot handle include order here
#include <QDesktopServices> // NOLINT cpplint cannot handle include order here
#include <QDir>             // NOLINT cpplint cannot handle include order here
#include <QFile>            // NOLINT cpplint cannot handle include order here
#include <QFileDialog>      // NOLINT cpplint cannot handle include order here
#include <QHBoxLayout>      // NOLINT cpplint cannot handle include order here
#include <QMenu>            // NOLINT cpplint cannot handle include order here
#include <QMenuBar>         // NOLINT cpplint cannot handle include order here
#include <QMessageBox>      // NOLINT cpplint cannot handle include order here
#include <QShortcut>        // NOLINT cpplint cannot handle include order here
#include <QSplashScreen>    // NOLINT cpplint cannot handle include order here
#include <QStatusBar>       // NOLINT cpplint cannot handle include order here
#include <QTimer>           // NOLINT cpplint cannot handle include order here
#include <QToolBar>         // NOLINT cpplint cannot handle include order here
#include <QToolButton>      // NOLINT cpplint cannot handle include order here
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/yaml_config_writer.hpp>
#include <rviz_rendering/render_window.hpp>
#include <string>
#include <utility>

#define CONFIG_EXTENSION "myrviz222"
#define CONFIG_EXTENSION_WILDCARD "*." CONFIG_EXTENSION
#define RECENT_CONFIG_COUNT 10

namespace rviz_common {

VisualizationFrame3::VisualizationFrame3(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
                                         QWidget *parent)
    : VisualizationFrame(rviz_ros_node, parent) {
  dock_manager_ = new ads::CDockManager();
}

void VisualizationFrame3::closeEvent(QCloseEvent *event) {
  VisualizationFrame::closeEvent(event);
  dock_manager_->deleteLater();
}

void VisualizationFrame3::loadWindowGeometry(const Config &config) {
  QString ads_dock_state;
  if (config.mapGetString("Ads dock State", &ads_dock_state)) {
    dock_manager_->restoreState(QByteArray::fromHex(qPrintable(ads_dock_state)));
    // qDebug() << " load state = " << qPrintable(ads_dock_state);
  }
  VisualizationFrame::loadWindowGeometry(config);
}

void VisualizationFrame3::saveWindowGeometry(Config config) {
  QByteArray ads_dock_state = dock_manager_->saveState().toHex();
  config.mapSetValue("Ads dock State", ads_dock_state.constData());
  VisualizationFrame::saveWindowGeometry(config);
}

void VisualizationFrame3::loadPanels(const Config &config) {
  // for (int i = 0; i < custom_panels_.size(); i++) {
  //   custom_panels_[i].dock_widget_->toggleViewAction()->deleteLater();
  //   dock_manager_->removeDockWidget(custom_panels_[i].dock_widget_);
  //   delete custom_panels_[i].dock;
  //   delete custom_panels_[i].delete_action;
  // }
  VisualizationFrame::loadPanels(config);
}

QDockWidget *VisualizationFrame3::addPanelByName(const QString &name,
                                                 const QString &class_id,
                                                 Qt::DockWidgetArea area,
                                                 bool floating) {
  return VisualizationFrame::addPanelByName(name, class_id, area, floating);
}

PanelDockWidget *
VisualizationFrame3::addPane(const QString &name, QWidget *panel, Qt::DockWidgetArea area, bool floating) {
  return VisualizationFrame::addPane(name, panel, area, floating);
}

void VisualizationFrame3::initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
                                     const QString &display_config_file) {
  VisualizationFrame::initialize(rviz_ros_node, display_config_file);

  // auto *CentralDockWidget = new ads::CDockWidget("CentralWidget");
  // CentralDockWidget->setWidget(render_panel_);
  // // auto *CentralDockArea = dock_manager_->addDockWidget(ads::LeftDockWidgetArea, CentralDockWidget);
  // dock_manager_->addDockWidget(ads::LeftDockWidgetArea, CentralDockWidget);
}

} // namespace rviz_common
