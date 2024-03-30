#include "plot/precision_display.h"
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/visualization_manager.hpp>

#include "plot/precision_widget.h"
#include "protobuf_helper.h"

PrecisionDisplay::PrecisionDisplay()
{
  using namespace rviz_common::properties;

  InitPersons();

  view_ = new PrecisionWidget();
  view_->setDisplaySync(this);

  // 绘制类型
  plot_type_prop_ = new EnumProperty("plot type", "Trajectory", "something like rtkplot", this, SLOT(UpdatePlotType()));
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

void PrecisionDisplay::load(const rviz_common::Config &config)
{
  rviz_common::Display::load(config);
  // qDebug() << "end graph size = " << graphs_.size();
}
void PrecisionDisplay::save(rviz_common::Config config) const
{
  rviz_common::Display::save(config);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PrecisionDisplay, rviz_common::Display)