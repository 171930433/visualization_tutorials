#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include "data_table_widget.h"

class QLineEdit;
class DataTableDisplay;

namespace rviz_plugin_tutorials
{

  class DataTablePanel : public rviz::Panel
  {
    // This class uses Qt slots and is a subclass of QObject, so it needs
    // the Q_OBJECT macro.
    Q_OBJECT
  public:
    DataTablePanel(QWidget *parent = 0);

    void onInitialize() override;
    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load(const rviz::Config &config);
    virtual void save(rviz::Config config) const;

    // Next come a couple of public Qt slots.
  public Q_SLOTS:

    // Here we declare some internal slots.
  protected Q_SLOTS:

  protected:

    DataTableWidget *data_table_;
    std::vector<MyStruct> datas_;

    //
    DataTableDisplay *raw_data_display_ = nullptr;
  };

} // end namespace rviz_plugin_tutorials

