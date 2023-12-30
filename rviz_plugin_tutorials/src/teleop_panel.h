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
#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H
// #include "qcustomplot.h"

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif
#include "qcustomplot.h"

#include "data_table_widget.h"

class QLineEdit;
class QCustomPlot;

class QCP_LIB_DECL QCPMapAxisTickerFixed : public QCPAxisTickerFixed
{
  Q_GADGET
public:
  QCPMapAxisTickerFixed(QCPAxis *x_axis, QCPAxis *y_axis) : QCPAxisTickerFixed()
  {
    x_axis_ = x_axis;
    y_axis_ = y_axis;
    //    this->setTickLength(0, 0);
  }

protected:
  // reimplemented virtual methods: range in meter
  virtual double getTickStep(const QCPRange &range) Q_DECL_OVERRIDE
  {
    y_axis_->setScaleRatio(x_axis_);

    double re = 0;

    if (x_axis_->range().size() >= y_axis_->range().size())
    {
      re = CalcStep(y_axis_->range().size(), y_axis_->axisRect()->height());
    }
    else
    {
      re = CalcStep(x_axis_->range().size(), x_axis_->axisRect()->width());
    }

    // qDebug() << "synced_ = " << synced_ << "getTickStep range = " << range << " step = " << re;

    return re;
  }

  virtual int getSubTickCount(double tickStep) Q_DECL_OVERRIDE { return 0; }

  double CalcStep(double const rangle_meter, int const range_pixel)
  {
    double step = 10;
    double pixel_per_meter = rangle_meter / range_pixel;
    double t[] = {1.0, 2.0, 5.0, 10.0}, tick = 30.0 * pixel_per_meter;
    double order = pow(10.0, floor(log10(tick)));
    for (int i = 0; i < 4; i++)
    {
      if (tick <= t[i] * order)
      {
        step = t[i] * order;
        break;
      }
    }
    return step;
  }

private:
  QCPAxis *x_axis_;
  QCPAxis *y_axis_;
};

class DockWidgetEventFilter : public QObject
{
protected:
  bool eventFilter(QObject *obj, QEvent *event) override
  {
    if (event->type() == QEvent::Type::WindowActivate)
    {
      QDockWidget *dockWidget = qobject_cast<QDockWidget *>(obj);
      if (dockWidget && dockWidget->isFloating())
      {
        if (setted_.count(dockWidget) == 0)
        {
          setted_[dockWidget] = false;
        }

        if (!setted_[dockWidget])
        {
          dockWidget->setWindowFlags(Qt::Window);
          dockWidget->show();
          setted_[dockWidget] = true;
          qDebug() << ros::Time::now().toNSec() << " " << obj->objectName() << " floated " << dockWidget->windowFlags();
        }
        // else
        // {
        //   event->accept();
        //   return true;
        // }
      }
      else
      {
        setted_[dockWidget] = false;
      }
    }
    return QObject::eventFilter(obj, event);
  }
  std::map<QDockWidget *, bool> setted_;
};

namespace rviz_plugin_tutorials
{

  class DriveWidget;

  // BEGIN_TUTORIAL
  // Here we declare our new subclass of rviz::Panel.  Every panel which
  // can be added via the Panels/Add_New_Panel menu is a subclass of
  // rviz::Panel.
  //
  // TeleopPanel will show a text-entry field to set the output topic
  // and a 2D control area.  The 2D control area is implemented by the
  // DriveWidget class, and is described there.
  class TeleopPanel : public rviz::Panel
  {
    // This class uses Qt slots and is a subclass of QObject, so it needs
    // the Q_OBJECT macro.
    Q_OBJECT
  public:
    // QWidget subclass constructors usually take a parent widget
    // parameter (which usually defaults to 0).  At the same time,
    // pluginlib::ClassLoader creates instances by calling the default
    // constructor (with no arguments).  Taking the parameter and giving
    // a default of 0 lets the default constructor work and also lets
    // someone using the class for something else to pass in a parent
    // widget as they normally would with Qt.
    TeleopPanel(QWidget *parent = 0);

    void onInitialize() override;
    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load(const rviz::Config &config);
    virtual void save(rviz::Config config) const;

    // Next come a couple of public Qt slots.
  public Q_SLOTS:
    // The control area, DriveWidget, sends its output to a Qt signal
    // for ease of re-use, so here we declare a Qt slot to receive it.
    void setVel(float linear_velocity_, float angular_velocity_);

    // In this example setTopic() does not get connected to any signal
    // (it is called directly), but it is easy to define it as a public
    // slot instead of a private function in case it would be useful to
    // some other user.
    void setTopic(const QString &topic);

    // Here we declare some internal slots.
  protected Q_SLOTS:
    // sendvel() publishes the current velocity values to a ROS
    // topic.  Internally this is connected to a timer which calls it 10
    // times per second.
    void sendVel();

    // updateTopic() reads the topic name from the QLineEdit and calls
    // setTopic() with the result.
    void updateTopic();

    // Then we finish up with protected member variables.
  protected:
    // The control-area widget which turns mouse events into command
    // velocities.
    // DriveWidget* drive_widget_;

    // One-line text editor for entering the outgoing ROS topic name.
    QLineEdit *output_topic_editor_;

    // The current name of the output topic.
    QString output_topic_;

    // The ROS publisher for the command velocity.
    ros::Publisher velocity_publisher_;

    // The ROS node handle.
    ros::NodeHandle nh_;

    // The latest velocity values from the drive widget.
    float linear_velocity_;
    float angular_velocity_;
    // END_TUTORIAL

    QCustomPlot *plot_;
    void setupTrajectoryDemo(QCustomPlot *customPlot);
    
    DataTableWidget *data_table_;
    std::vector<MyStruct> datas_;
  };

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
