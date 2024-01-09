#include "plot/trajectory_display.h"
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/visualization_manager.h>

#include "plot/trajectory_widget.h"
#include "protobuf_helper.h"

int GraphProperty::graph_counts_ = 0;

GraphProperty::~GraphProperty()
{
  graph_counts_--;
}

void GraphProperty::SyncInfo()
{
  // 去buffer里面查询通道更新
  // 去buffer里面查询数据更新
}

GraphProperty::GraphProperty(TrajectoryWidget *plot, Property *parent)
    : rviz::BoolProperty(QString("curve-%1").arg(graph_counts_++), true, "options", parent),
      plot_(plot)
{
  setDisableChildrenIfFalse(true);
  connect(this, SIGNAL(changed()), this, SLOT(UpdateEnable()));

  channel_name_prop_ = new rviz::EnumProperty("topic name", "None", "the topic of trajectory", this, SLOT(UpdateTopic()));
  channel_name_prop_->addOption("None", 0);
  channel_name_prop_->addOption("random-0", 1);
  channel_name_prop_->addOption("random-1", 2);

  // scatter
  scatter_type_ = new rviz::EnumProperty("Point type", "None", "the point type of trajectory", this, SLOT(UpdateScatterShape()));
  scatter_type_->addOption("None", QCPScatterStyle::ScatterShape::ssNone);
  scatter_type_->addOption("ssDot", QCPScatterStyle::ScatterShape::ssDot);
  scatter_type_->addOption("ssCross", QCPScatterStyle::ScatterShape::ssCross);
  scatter_type_->addOption("ssPlus", QCPScatterStyle::ScatterShape::ssPlus);
  scatter_type_->addOption("ssCircle", QCPScatterStyle::ScatterShape::ssCircle);
  scatter_type_->addOption("ssDisc", QCPScatterStyle::ScatterShape::ssDisc);
  scatter_type_->addOption("ssSquare", QCPScatterStyle::ScatterShape::ssSquare);
  scatter_type_->addOption("ssDiamond", QCPScatterStyle::ScatterShape::ssDiamond);
  scatter_type_->addOption("ssStar", QCPScatterStyle::ScatterShape::ssStar);
  scatter_type_->addOption("ssTriangle", QCPScatterStyle::ScatterShape::ssTriangle);
  // scatter_type_->addOption("ssTriangleInverted", QCPScatterStyle::ScatterShape::ssTriangleInverted);
  // scatter_type_->addOption("ssCrossSquare", QCPScatterStyle::ScatterShape::ssCrossSquare);
  // scatter_type_->addOption("ssPlusSquare", QCPScatterStyle::ScatterShape::ssPlusSquare);
  // scatter_type_->addOption("ssCrossCircle", QCPScatterStyle::ScatterShape::ssCrossCircle);
  // scatter_type_->addOption("ssPlusCircle", QCPScatterStyle::ScatterShape::ssPlusCircle);
  // scatter_type_->addOption("ssPeace", QCPScatterStyle::ScatterShape::ssPeace);
  // scatter_type_->addOption("ssPixmap", QCPScatterStyle::ScatterShape::ssPixmap);
  // scatter_type_->addOption("ssCustom", QCPScatterStyle::ScatterShape::ssCustom);

  scatter_color_ = new rviz::ColorProperty("scatter color", QColor(Qt::blue), "set the color of all scatter", this, SLOT(UpdateScatterColor()));

  scatter_size_ = new rviz::IntProperty("scatter size", 1, "the size of all scatter", this, SLOT(UpdateScatterSize()));
  scatter_size_->setMin(1);
  scatter_size_->setMax(10);
  // line
  line_type_ = new rviz::EnumProperty("line type", "Line", "the line type of trajectory", this, SLOT(UpdateLineStyle()));
  line_type_->addOption("None", QCPCurve::LineStyle::lsNone);
  line_type_->addOption("Line", QCPCurve::LineStyle::lsLine);

  line_color_ = new rviz::ColorProperty("line color", QColor(Qt::gray), "set the color of line", this, SLOT(UpdateLineColor()));
  line_width_ = new rviz::IntProperty("line width", 1, "the line width of trajectory", this, SLOT(UpdateLineWidth()));
  line_width_->setMin(1);
  line_width_->setMax(10);

  // 数据通道更新，数据更新
  connect(&dataTimer_, SIGNAL(timeout()), this, SLOT(SyncInfo()));
  dataTimer_.start(500); // Interval 0 means to refresh as fast as possible
}
void GraphProperty::UpdateTopic()
{
  QString name = channel_name_prop_->getValue().toString();
  // 删除当前轨迹
  if (channel_name_prop_->getOptionInt() == 0)
  {
    plot_->RemoveCurve(curve_);
  }
  else
  {
    auto *re = plot_->ContainsCurve(name);
    curve_ = (re ? re : plot_->addTrajectory(name, g_messages, getScatterStyle(), getLinePen()));
  }
  plot_->replot();
}

QPen GraphProperty::getLinePen() const
{
  QPen line_pen;
  line_pen.setColor(line_color_->getColor());
  line_pen.setWidth(line_width_->getInt());
  return line_pen;
}
QCPScatterStyle GraphProperty::getScatterStyle() const
{
  auto const shape = static_cast<QCPScatterStyle::ScatterShape>(scatter_type_->getOptionInt());
  auto const color = scatter_color_->getColor();
  double const size = scatter_size_->getInt();

  return QCPScatterStyle{shape, color, size};
}
void GraphProperty::UpdateEnable()
{
  if (!curve_)
    return;
  curve_->setVisible(this->getBool());
  plot_->replot();
  // qDebug() << " GraphProperty::UpdateEnable() called " << this->getBool();
}
void GraphProperty::UpdateScatterSize()
{
  if (!curve_)
    return;
  auto const size = scatter_size_->getInt();
  auto new_scatter_style = curve_->scatterStyle();
  new_scatter_style.setSize(size);
  curve_->setScatterStyle(new_scatter_style);
  plot_->replot();
}

void GraphProperty::UpdateScatterShape()
{
  if (!curve_)
    return;
  auto const type = static_cast<QCPScatterStyle::ScatterShape>(scatter_type_->getOptionInt());
  auto new_scatter_style = curve_->scatterStyle();
  new_scatter_style.setShape(type);
  curve_->setScatterStyle(new_scatter_style);
  plot_->replot();
}
void GraphProperty::UpdateScatterColor()
{
  if (!curve_)
    return;
  auto const new_color = scatter_color_->getColor();
  auto new_scatter_style = curve_->scatterStyle();
  QPen new_pen = new_scatter_style.pen();
  new_pen.setColor(new_color);
  new_scatter_style.setPen(new_pen);
  curve_->setScatterStyle(new_scatter_style);
  plot_->replot();
}
void GraphProperty::UpdateLineStyle()
{
  if (!curve_)
    return;
  auto const type = static_cast<QCPCurve::LineStyle>(line_type_->getOptionInt());
  line_width_->setHidden(type == QCPCurve::LineStyle::lsNone ? true : false);
  line_color_->setHidden(type == QCPCurve::LineStyle::lsNone ? true : false);
  curve_->setLineStyle(type);
  plot_->replot();
}

void GraphProperty::UpdateLineColor()
{
  if (!curve_)
    return;
  auto const new_color = line_color_->getColor();
  QPen new_pen = curve_->pen();
  new_pen.setColor(new_color);
  curve_->setPen(new_pen);
  plot_->replot();
}

void GraphProperty::UpdateLineWidth()
{
  if (!curve_)
    return;
  auto const new_width = line_width_->getInt();
  QPen new_pen = curve_->pen();
  new_pen.setWidthF(new_width);
  curve_->setPen(new_pen);
  plot_->replot();
}

TrajectoryDisplay::TrajectoryDisplay()
{
  InitPersons();
  view_ = new TrajectoryWidget();
  view_->setDisplaySync(this);

  // swap2central_ = new rviz::BoolProperty("Set in central", false, "swap the trajectory and render view", this, SLOT(Swap2Central()));
  focus_when_select_ = new rviz::BoolProperty("foucs when select", true, "focus the selected points", this, SLOT(UpdateFocusWhenSelect()));
  counts_prop_ = new rviz::IntProperty("graph counts", 1, "the number of graph counts", this, SLOT(UpdateGraphCount()));
  counts_prop_->setMin(1);
  counts_prop_->setMax(10);
  for (int i = 0; i < counts_prop_->getValue().toInt(); ++i)
  {
    graphs_.push_back(std::make_shared<GraphProperty>(view_, this));
  }
}

TrajectoryDisplay::~TrajectoryDisplay()
{
  if (initialized())
  {
    delete view_;
    graphs_.clear();
  }
}

// Overrides from Display
void TrajectoryDisplay::onInitialize()
{
  setAssociatedWidget(view_);
}

void TrajectoryDisplay::update(float dt, float ros_dt)
{
}

void TrajectoryDisplay::UpdateFocusWhenSelect()
{
  view_->setFocusWhenSelect(focus_when_select_->getBool());
}

void TrajectoryDisplay::UpdateGraphCount()
{
  int const new_count = counts_prop_->getInt();
  int const old_count = graphs_.size();

  if (new_count < old_count) // 删除元素
  {
    for (int i = new_count; i < old_count; ++i)
    {
      this->takeChild(graphs_.back().get());
      graphs_.pop_back();
    }
  }
  else
  {
    for (int i = old_count; i < new_count; ++i)
    {
      graphs_.push_back(std::make_shared<GraphProperty>(view_, this));
    }
  }
  // qDebug() <<" UpdateGraphCount called";
}

void TrajectoryDisplay::load(const rviz::Config &config)
{
//   qDebug() << config.getValue();
// qDebug() << "before graph size = " << graphs_.size();

  rviz::Display::load(config);
// qDebug() << "end graph size = " << graphs_.size();

}
void TrajectoryDisplay::save(rviz::Config config) const
{
  rviz::Display::save(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrajectoryDisplay, rviz::Display)