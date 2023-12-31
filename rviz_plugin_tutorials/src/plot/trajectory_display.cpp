#include "plot/trajectory_display.h"
#include <rviz/properties/int_property.h>

#include "plot/trajectory_widget.h"

TrajectoryDisplay::TrajectoryDisplay()
{
}
TrajectoryDisplay::~TrajectoryDisplay()
{
}

// Overrides from Display
void TrajectoryDisplay::onInitialize()
{
    main_interval_ = new rviz::IntProperty("interval", 100, "main grid interval[5,1000]", this, SLOT(UpdateInterval()));
    main_interval_->setMin(5);
    main_interval_->setMax(1000);
    sub_range_ = new rviz::IntProperty("range", 100, "sub grid range [10,100]", this, SLOT(UpdateRange()));
    sub_range_->setMin(10);
    sub_range_->setMax(100);
}

void TrajectoryDisplay::UpdateInterval()
{
    // view_->setMainInterval(main_interval_->getInt());
}
void TrajectoryDisplay::UpdateRange()
{
    // view_->setSubRange(sub_range_->getInt());
}

void TrajectoryDisplay::update(float dt, float ros_dt)
{
}