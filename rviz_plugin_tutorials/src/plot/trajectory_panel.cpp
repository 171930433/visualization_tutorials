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

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <rviz/visualization_manager.h>

#include "plot/trajectory_panel.h"
#include "plot/trajectory_widget.h"
#include "plot/trajectory_display.h"

// swap to central
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_frame.h>
#include <rviz/window_manager_interface.h>

namespace zhito
{

  TrajectoryPanel::~TrajectoryPanel()
  {
  }

  void TrajectoryPanel::UpdateView()
  {
    if (!plot_ || !plot_->isVisible())
    {
      return;
    }
    plot_->SyncDataAndView();
  }

  TrajectoryPanel::TrajectoryPanel(QWidget *parent) : rviz::Panel(parent)
  {
    plot_ = new TrajectoryWidget(this);
    plot_->setVisible(true);

    // Lay out the topic field above the control widget.
    v_layout_ = new QVBoxLayout;
    // v_layout_->addLayout(topic_layout);
    v_layout_->addWidget(plot_);
    setLayout(v_layout_);
  }

  // Save all configuration data from this panel to the given
  // Config object.  It is important here that you call save()
  // on the parent class so the class id and panel name get saved.
  void TrajectoryPanel::save(rviz::Config config) const { rviz::Panel::save(config); }

  // Load all configuration data for this panel from the given Config object.
  void TrajectoryPanel::load(const rviz::Config &config) { rviz::Panel::load(config); }

  void TrajectoryPanel::onInitialize()
  {
    //
    // raw_data_display_ = vis_manager_->createDisplay("rviz/Grid", "DataGrid",true);
    raw_data_display_ = new TrajectoryDisplay();
    // 给view
    raw_data_display_->setView(plot_);
    raw_data_display_->setPanel(this);
    raw_data_display_->setName(QString("Trajectory"));
    vis_manager_->addDisplay(raw_data_display_, true);

    // 当前panel可以最大化
    auto main_window = vis_manager_->getWindowManager()->getParentWindow();
    QList<QDockWidget *> dockWidgets = main_window->findChildren<QDockWidget *>();
    for (QDockWidget *dockWidget : dockWidgets)
    {
      if (dockWidget->widget() == this)
      {
        qDebug() << dockWidget->windowTitle() << " install DockWidgetEventFilter ";
        dockWidget->installEventFilter(new DockWidgetEventFilter());
      }
    }
  }

  void TrajectoryPanel::Swap2Central(bool insert)
  {
    auto main_window = dynamic_cast<rviz::VisualizationFrame *>(vis_manager_->getWindowManager());
    auto cw_layoyt = main_window->centralWidget()->layout();

    if (insert)
    {
      qobject_cast<QBoxLayout *>(cw_layoyt)->insertWidget(1, plot_, 1);
      v_layout_->insertWidget(0, vis_manager_->getRenderPanel(), 1);
    }
    else
    {
      qobject_cast<QBoxLayout *>(cw_layoyt)->insertWidget(1, vis_manager_->getRenderPanel(), 1);
      v_layout_->insertWidget(0, plot_, 1);
    }

    // 切换主窗口
    // this->layout()->addWidget();
    // this->layout()->update();
    //

    // plot_->setEnabled(false);
    // main_window->takeCentralWidget();
    // main_window->setCentralWidget(plot_);

    // main_window->dock
    // cw_layoyt->takeAt(1);
    cw_layoyt->update();
    v_layout_->update();
    // plot_->setVisible(true);
    // main_window->centralWidget()->layout()->update();
  }

} // namespace zhito

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zhito::TrajectoryPanel, rviz::Panel)
// END_TUTORIAL
