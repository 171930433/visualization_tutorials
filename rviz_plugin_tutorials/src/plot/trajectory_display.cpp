#include "plot/trajectory_display.h"
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>

#include "plot/trajectory_widget.h"
#include "plot/trajectory_panel.h"

TrajectoryDisplay::TrajectoryDisplay()
{
}
TrajectoryDisplay::~TrajectoryDisplay()
{
}

// Overrides from Display
void TrajectoryDisplay::onInitialize()
{
    // main_interval_ = new rviz::IntProperty("interval", 100, "main grid interval[5,1000]", this, SLOT(UpdateInterval()));
    // main_interval_->setMin(5);
    // main_interval_->setMax(1000);
    // sub_range_ = new rviz::IntProperty("range", 100, "sub grid range [10,100]", this, SLOT(UpdateRange()));
    // sub_range_->setMin(10);
    // sub_range_->setMax(100);

    swap2central_ = new rviz::BoolProperty("Set in central", false, "swap the trajectory and render view", this, SLOT(Swap2Central()));

    scatter_type_ = new rviz::EnumProperty("Point type", "Cross", "the point type of trajectory", this, SLOT(UpdateScatterShape()));
    scatter_type_->addOption("Circle", QCPScatterStyle::ScatterShape::ssCircle);
    scatter_type_->addOption("Cross", QCPScatterStyle::ScatterShape::ssCross);
    scatter_type_->addOption("CrossCircle", QCPScatterStyle::ScatterShape::ssCrossCircle);
    scatter_type_->addOption("CrossSquare", QCPScatterStyle::ScatterShape::ssCrossSquare);
    scatter_type_->addOption("Diamond", QCPScatterStyle::ScatterShape::ssDiamond);

    line_type_ = new rviz::EnumProperty("line type", "Line", "the line type of trajectory", this, SLOT(UpdateLineStyle()));
    line_type_->addOption("None", QCPGraph::LineStyle::lsNone);
    line_type_->addOption("Line", QCPGraph::LineStyle::lsLine);
}

void TrajectoryDisplay::UpdateScatterShape()
{
    auto const type = static_cast<QCPScatterStyle::ScatterShape>(scatter_type_->getOptionInt());
    // view_->setMainInterval(main_interval_->getInt());
    view_->ChangeScatterShape(type);
}

void TrajectoryDisplay::UpdateLineStyle()
{
    auto const type = static_cast<QCPGraph::LineStyle>(line_type_->getOptionInt());
    // view_->setMainInterval(main_interval_->getInt());
    view_->ChangeLineStyle(type);
}

void TrajectoryDisplay::update(float dt, float ros_dt)
{
}

void TrajectoryDisplay::Swap2Central()
{
    panel_->Swap2Central(swap2central_->getBool());
}