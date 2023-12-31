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

#include "data_table_panel.h"

// #include "drive_widget.h"

#include <rviz/visualization_manager.h>
#include <rviz/display_group.h>

#include "data_table_display.h"

namespace rviz_plugin_tutorials
{

  DataTablePanel::DataTablePanel(QWidget *parent)
      : rviz::Panel(parent)
  {
    data_table_ = new DataTableWidget;
    data_table_->setMainInterval(100);
    data_table_->setSubRange(200);

    // 构造数据
    int count = 100 * 10000;
    datas_.reserve(count);
    for (int i = 0; i < count; ++i)
    {
      datas_.emplace_back(MyStruct{i, i * 1.0, std::to_string(2 * i), Eigen::Vector3d::Identity().array() + i});
    }
    data_table_->setData(datas_);

    // Lay out the topic field above the control widget.
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(data_table_, 1);
    setLayout(layout);
  }

  void DataTablePanel::onInitialize()
  {
    //
    // raw_data_display_ = vis_manager_->createDisplay("rviz/Grid", "DataGrid",true);
    raw_data_display_ = new DataTableDisplay();
    // 给view
    raw_data_display_->setDataTableView(data_table_);
    raw_data_display_->setName(QString("DataTable"));
    vis_manager_->addDisplay(raw_data_display_, true);
  }

  // Save all configuration data from this panel to the given
  // Config object.  It is important here that you call save()
  // on the parent class so the class id and panel name get saved.
  void DataTablePanel::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }

  // Load all configuration data for this panel from the given Config object.
  void DataTablePanel::load(const rviz::Config &config)
  {
    rviz::Panel::load(config);
  }

} // end namespace rviz_plugin_tutorials

// 该 display不需要导出
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::DataTablePanel, rviz::Panel)
// END_TUTORIAL
