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
    raw_data_display_->setName(QString("Trajectory"));
    vis_manager_->addDisplay(raw_data_display_, true);
  }

} // namespace zhito

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zhito::TrajectoryPanel, rviz::Panel)
// END_TUTORIAL
