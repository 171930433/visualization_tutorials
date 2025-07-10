/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_COMMON__VISUALIZATION_FRAME3_HPP_
#define RVIZ_COMMON__VISUALIZATION_FRAME3_HPP_

#include <QList>       // NOLINT: cpplint is unable to handle the include order here
#include <QMainWindow> // NOLINT: cpplint is unable to handle the include order here
#include <QString>     // NOLINT: cpplint is unable to handle the include order here
#include <Qt>          // NOLINT: cpplint is unable to handle the include order here
#include <chrono>
#include <deque>
#include <map>
#include <rviz_common/config.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_rendering/render_window.hpp>
#include <string>

class QAction;
class QActionGroup;
class QApplication;
class QCloseEvent;
class QDockWidget;
class QEvent;
class QLabel;
class QSplashScreen;
class QTimer;
class QToolButton;
class QWidget;

namespace ads {
class CDockManager;
class CDockWidget;
} // namespace ads

namespace rviz_common {

class Panel;
class PanelDockWidget;
class PanelFactory;
class RenderPanel;
class Tool;
class VisualizationManager;
class WidgetGeometryChangeDetector;

/// The main rviz window.
/**
 * VisualizationFrame2 is a QMainWindow, which means it has a center area and a
 * bunch of dock areas around it.
 * The central widget here is a RenderPanel, and around it (by default) are the
 * DisplaysPanel, ViewsPanel, TimePanel, SelectionPanel, and
 * ToolPropertiesPanel.
 * At the top is a toolbar with Tools like "Move Camera", "Select", etc.
 * There is also a menu bar with file/open, etc.
 */
class RVIZ_COMMON_PUBLIC VisualizationFrame3 : public VisualizationFrame {
  Q_OBJECT
public:
  // using rviz_common::VisualizationFrame::VisualizationFrame;

  explicit VisualizationFrame3(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
                               QWidget *parent = nullptr);
  void initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
                  const QString &display_config_file = "");
  void closeEvent(QCloseEvent *event) override;
  void loadWindowGeometry(const Config &config);
  void saveWindowGeometry(Config config);
  void loadPanels(const Config &config);
  // !
  PanelDockWidget *addPane(const QString &name,
                           QWidget *panel,
                           Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
                           bool floating = true) override;

  QDockWidget *addPanelByName(const QString &name,
                              const QString &class_lookup_name,
                              Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
                              bool floating = true);

protected:
  // addedi
  ads::CDockManager *dock_manager_;
};

} // namespace rviz_common

#endif // RVIZ_COMMON__VISUALIZATION_FRAME3_HPP_
