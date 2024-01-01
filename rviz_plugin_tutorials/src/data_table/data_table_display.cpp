#include "data_table_display.h"
#include <rviz/properties/int_property.h>

#include "data_table/data_table_widget.h"

DataTableDisplay::DataTableDisplay()
{
}
DataTableDisplay::~DataTableDisplay()
{
}
ITimeSync *DataTableDisplay::getView() { return view_; }

// Overrides from Display
void DataTableDisplay::onInitialize()
{
    main_interval_ = new rviz::IntProperty("interval", 100, "main grid interval[5,1000]", this, SLOT(UpdateInterval()));
    main_interval_->setMin(5);
    main_interval_->setMax(1000);
    sub_range_ = new rviz::IntProperty("range", 100, "sub grid range [10,100]", this, SLOT(UpdateRange()));
    sub_range_->setMin(10);
    sub_range_->setMax(100);
}

void DataTableDisplay::UpdateInterval()
{
    view_->setMainInterval(main_interval_->getInt());
}
void DataTableDisplay::UpdateRange()
{
    view_->setSubRange(sub_range_->getInt());
}

void DataTableDisplay::update(float dt, float ros_dt)
{
}