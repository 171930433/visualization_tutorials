#include "plot/precision_display.h"
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/visualization_manager.h>

#include "plot/precision_widget.h"
#include "protobuf_helper.h"

PrecisionDisplay::PrecisionDisplay()
{
  InitPersons();

  view_ = new PrecisionWidget();
  view_->setDisplaySync(this);

  // 绘制类型
  plot_type_prop_ = new rviz::EnumProperty("plot type", "Trajectory", "something like rtkplot", this, SLOT(UpdatePlotType()));
  plot_type_prop_->addOption("Trajectory", 0);
  plot_type_prop_->addOption("Position", 1);
  plot_type_prop_->addOption("Velocity", 2);
}

PrecisionDisplay::~PrecisionDisplay()
{
  if (initialized())
  {
    delete view_;
  }
}

// Overrides from Display
void PrecisionDisplay::onInitialize()
{
  setAssociatedWidget(view_);
}

void PrecisionDisplay::update(float dt, float ros_dt)
{
}

void PrecisionDisplay::load(const rviz::Config &config)
{
  rviz::Display::load(config);
  // qDebug() << "end graph size = " << graphs_.size();
}
void PrecisionDisplay::save(rviz::Config config) const
{
  rviz::Display::save(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PrecisionDisplay, rviz::Display)