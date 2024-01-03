#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <memory>
// #include "plot/plot_base.h"

class TrajectoryWidget;
class TrajectoryDisplay;

namespace zhito
{
  // namespace lm {
  // class LMComponent;
  // }


  class TrajectoryPanel : public rviz::Panel
  {
    // This class uses Qt slots and is a subclass of QObject, so it needs
    // the Q_OBJECT macro.
    Q_OBJECT
  public:
    TrajectoryPanel(QWidget *parent = 0);
    ~TrajectoryPanel() override;

    void UpdateView();

    void onInitialize() override;

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load(const rviz::Config &config);
    virtual void save(rviz::Config config) const;

    // Next come a couple of public Qt slots.
  public Q_SLOTS:
    void Swap2Central(bool insert = false);
    // Here we declare some internal slots.
  protected Q_SLOTS:

    // Then we finish up with protected member variables.
  protected:
    // END_TUTORIAL

    QVBoxLayout *v_layout_ = nullptr;

    TrajectoryWidget* plot_ = nullptr;
    TrajectoryDisplay *raw_data_display_ = nullptr;

  private slots:
  };

} // namespace zhito

