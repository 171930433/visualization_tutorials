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

#include "qcustomplot.h"

#include <geometry_msgs/Twist.h>
#include "teleop_panel.h"

// #include "drive_widget.h"

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_frame.h>
#include <rviz/window_manager_interface.h>

namespace rviz_plugin_tutorials
{

  void TeleopPanel::setupTrajectoryDemo(QCustomPlot *customPlot)
  {
    // demoName = "Vector3 Demo";

    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    customPlot->xAxis->setRange(-8, 8);
    customPlot->yAxis->setRange(-5, 5);
    customPlot->axisRect()->setupFullAxesBox();

    // xy with same scale strategy
    QSharedPointer<QCPMapAxisTickerFixed> map_ticker;
    map_ticker = QSharedPointer<QCPMapAxisTickerFixed>::create(customPlot->xAxis, customPlot->yAxis);
    map_ticker->setTickStep(10.0);
    map_ticker->setScaleStrategy(QCPAxisTickerFixed::ssNone);

    foreach (QCPAxis *axis, customPlot->axisRect()->axes())
    {
      axis->setTicker(map_ticker);
      axis->setTickLength(0, 0);
      axis->setTickLabels(false);
    }

    customPlot->legend->setVisible(true);
    QFont legendFont = QWidget::font();
    legendFont.setPointSize(10);
    customPlot->legend->setFont(legendFont);
    customPlot->legend->setSelectedFont(legendFont);
    customPlot->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items

    // xy axis with same scale factor
    customPlot->yAxis->setScaleRatio(customPlot->xAxis, 1.0);

    customPlot->rescaleAxes();

    // connect slot that shows a message in the status bar when a graph is clicked:
    // connect(customPlot, SIGNAL(plottableClick(QCPAbstractPlottable *, int, QMouseEvent *)), this, SLOT(graphClicked(QCPAbstractPlottable *, int)));

    // setup policy and connect slot for context menu popup:
    // customPlot->setContextMenuPolicy(Qt::CustomContextMenu);
    // connect(customPlot, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));
  }

  // BEGIN_TUTORIAL
  // Here is the implementation of the TeleopPanel class.  TeleopPanel
  // has these responsibilities:
  //
  // - Act as a container for GUI elements DriveWidget and QLineEdit.
  // - Publish command velocities 10 times per second (whether 0 or not).
  // - Saving and restoring internal state from a config file.
  //
  // We start with the constructor, doing the standard Qt thing of
  // passing the optional *parent* argument on to the superclass
  // constructor, and also zero-ing the velocities we will be
  // publishing.
  TeleopPanel::TeleopPanel(QWidget *parent)
      : rviz::Panel(parent), linear_velocity_(0), angular_velocity_(0)
  {
    // Next we lay out the "output topic" text entry field using a
    // QLabel and a QLineEdit in a QHBoxLayout.
    QHBoxLayout *topic_layout = new QHBoxLayout;
    topic_layout->addWidget(new QLabel("Output Topic:"));
    output_topic_editor_ = new QLineEdit;
    topic_layout->addWidget(output_topic_editor_);

    // Then create the control widget.
    // drive_widget_ = new DriveWidget;

    data_table_ = new DataTableWidget;
    data_table_->setMainInterval(100);
    data_table_->setSubRange(200);

    // 构造数据
    int count = 100 * 10000;
    datas_.reserve(count);
    for (int i = 0; i < count; ++i)
    {
      datas_.emplace_back(MyStruct{i, i * 1.0, std::to_string(2 * i)});
    }
    data_table_->setData(datas_);

    // Lay out the topic field above the control widget.
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(topic_layout);
    // layout->addWidget(data_table_, 1);
    layout->addWidget(data_table_, 1);
    setLayout(layout);

    // Create a timer for sending the output.  Motor controllers want to
    // be reassured frequently that they are doing the right thing, so
    // we keep re-sending velocities even when they aren't changing.
    //
    // Here we take advantage of QObject's memory management behavior:
    // since "this" is passed to the new QTimer as its parent, the
    // QTimer is deleted by the QObject destructor when this TeleopPanel
    // object is destroyed.  Therefore we don't need to keep a pointer
    // to the timer.
    QTimer *output_timer = new QTimer(this);

    // Next we make signal/slot connections.
    // connect(drive_widget_, SIGNAL(outputVelocity(float, float)), this, SLOT(setVel(float, float)));
    connect(output_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
    connect(output_timer, SIGNAL(timeout()), this, SLOT(sendVel()));

    // Start the timer.
    output_timer->start(100);

    // Make the control widget start disabled, since we don't start with an output topic.
    // drive_widget_->setEnabled(false);
  }

  void TeleopPanel::onInitialize()
  {
    return;
    // 切换主窗口
    this->layout()->addWidget(vis_manager_->getRenderPanel());
    this->layout()->update();
    //
    plot_ = new QCustomPlot();
    setupTrajectoryDemo(plot_);
    // plot_->setEnabled(false);
    auto main_window = dynamic_cast<rviz::VisualizationFrame *>(vis_manager_->getWindowManager());
    // main_window->takeCentralWidget();
    // main_window->setCentralWidget(plot_);
    QList<QDockWidget *> dockWidgets = main_window->findChildren<QDockWidget *>();

    // 打印或处理 dockWidgets
    for (QDockWidget *dockWidget : dockWidgets)
    {
      if (dockWidget->widget() == this)
      {
        qDebug() << dockWidget->windowTitle() << " install DockWidgetEventFilter ";
        dockWidget->installEventFilter(new DockWidgetEventFilter());
      }
    }
    // main_window->dock
    auto cw_layoyt = main_window->centralWidget()->layout();
    // cw_layoyt->takeAt(1);
    qobject_cast<QBoxLayout *>(cw_layoyt)->insertWidget(1, plot_, 1);
    cw_layoyt->update();
    // plot_->setVisible(true);
    // main_window->centralWidget()->layout()->update();
  }

  // setVel() is connected to the DriveWidget's output, which is sent
  // whenever it changes due to a mouse event.  This just records the
  // values it is given.  The data doesn't actually get sent until the
  // next timer callback.
  void TeleopPanel::setVel(float lin, float ang)
  {
    linear_velocity_ = lin;
    angular_velocity_ = ang;
  }

  // Read the topic name from the QLineEdit and call setTopic() with the
  // results.  This is connected to QLineEdit::editingFinished() which
  // fires when the user presses Enter or Tab or otherwise moves focus
  // away.
  void TeleopPanel::updateTopic()
  {
    setTopic(output_topic_editor_->text());
  }

  // Set the topic name we are publishing to.
  void TeleopPanel::setTopic(const QString &new_topic)
  {
    // Only take action if the name has changed.
    if (new_topic != output_topic_)
    {
      output_topic_ = new_topic;
      // If the topic is the empty string, don't publish anything.
      if (output_topic_ == "")
      {
        velocity_publisher_.shutdown();
      }
      else
      {
        // The old ``velocity_publisher_`` is destroyed by this assignment,
        // and thus the old topic advertisement is removed.  The call to
        // nh_advertise() says we want to publish data on the new topic
        // name.
        velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(output_topic_.toStdString(), 1);
      }
      // rviz::Panel defines the configChanged() signal.  Emitting it
      // tells RViz that something in this panel has changed that will
      // affect a saved config file.  Ultimately this signal can cause
      // QWidget::setWindowModified(true) to be called on the top-level
      // rviz::VisualizationFrame, which causes a little asterisk ("*")
      // to show in the window's title bar indicating unsaved changes.
      Q_EMIT configChanged();
    }

    // Gray out the control widget when the output topic is empty.
    // drive_widget_->setEnabled(output_topic_ != "");
  }

  // Publish the control velocities if ROS is not shutting down and the
  // publisher is ready with a valid topic name.
  void TeleopPanel::sendVel()
  {
    if (ros::ok() && velocity_publisher_)
    {
      geometry_msgs::Twist msg;
      msg.linear.x = linear_velocity_;
      msg.linear.y = 0;
      msg.linear.z = 0;
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = angular_velocity_;
      velocity_publisher_.publish(msg);
    }
  }

  // Save all configuration data from this panel to the given
  // Config object.  It is important here that you call save()
  // on the parent class so the class id and panel name get saved.
  void TeleopPanel::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
    config.mapSetValue("Topic", output_topic_);
  }

  // Load all configuration data for this panel from the given Config object.
  void TeleopPanel::load(const rviz::Config &config)
  {
    rviz::Panel::load(config);
    QString topic;
    if (config.mapGetString("Topic", &topic))
    {
      output_topic_editor_->setText(topic);
      updateTopic();
    }
  }

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::TeleopPanel, rviz::Panel)
// END_TUTORIAL
