#include "plot/trajectory_display.h"
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>

#include "plot/trajectory_widget.h"
#include "plot/trajectory_panel.h"

GraphProperty::GraphProperty(QCPCurve *graph, Property *parent)
    : rviz::BoolProperty(graph->name(), true, "options", parent),
      graph_(graph)
{
  setDisableChildrenIfFalse(true);
  connect(this, SIGNAL(changed()), this, SLOT(UpdateEnable()));

  scatter_type_ = new rviz::EnumProperty("Point type", "Cross", "the point type of trajectory", this, SLOT(UpdateScatterShape()));
  scatter_type_->addOption("Circle", QCPScatterStyle::ScatterShape::ssCircle);
  scatter_type_->addOption("Cross", QCPScatterStyle::ScatterShape::ssCross);
  scatter_type_->addOption("CrossCircle", QCPScatterStyle::ScatterShape::ssCrossCircle);
  scatter_type_->addOption("CrossSquare", QCPScatterStyle::ScatterShape::ssCrossSquare);
  scatter_type_->addOption("Diamond", QCPScatterStyle::ScatterShape::ssDiamond);

  line_type_ = new rviz::EnumProperty("line type", "Line", "the line type of trajectory", this, SLOT(UpdateLineStyle()));
  line_type_->addOption("None", QCPCurve::LineStyle::lsNone);
  line_type_->addOption("Line", QCPCurve::LineStyle::lsLine);
}
void GraphProperty::UpdateEnable()
{
  graph_->setVisible(this->getBool());
  graph_->parentPlot()->replot();
  // qDebug() << " GraphProperty::UpdateEnable() called " << this->getBool();
}

void GraphProperty::UpdateScatterShape()
{
  auto const type = static_cast<QCPScatterStyle::ScatterShape>(scatter_type_->getOptionInt());
  graph_->setScatterStyle(type);
  graph_->parentPlot()->replot();
}

void GraphProperty::UpdateLineStyle()
{
  auto const type = static_cast<QCPCurve::LineStyle>(line_type_->getOptionInt());
  // view_->setMainInterval(main_interval_->getInt());
  graph_->setLineStyle(type);
  graph_->parentPlot()->replot();
}
TrajectoryDisplay::TrajectoryDisplay()
{
}
TrajectoryDisplay::~TrajectoryDisplay()
{
}
ITimeSync *TrajectoryDisplay::getView() { return view_; }

// Overrides from Display
void TrajectoryDisplay::onInitialize()
{
  swap2central_ = new rviz::BoolProperty("Set in central", false, "swap the trajectory and render view", this, SLOT(Swap2Central()));
}

void TrajectoryDisplay::update(float dt, float ros_dt)
{
  // int const count = view_->graphCount();
  // for (int i = 0; i < count; ++i)
  // {
  //     auto *graph = view_->graph(i);
  // }

  for (auto &[name, curve] : view_->Curves())
  {
    if (graphs_.count(name) <= 0)
    {
      graphs_[name] = new GraphProperty(curve, this);
    }
  }
}

void TrajectoryDisplay::Swap2Central()
{
  panel_->Swap2Central(swap2central_->getBool());
}