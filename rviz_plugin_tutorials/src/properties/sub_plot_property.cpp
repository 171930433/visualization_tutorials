#include "sub_plot_property.h"
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/int_property.hpp>
namespace rviz_common {
namespace properties {

PlotableProperty::PlotableProperty(Property *parent) {
  // scatter
  scatter_type_ =
      new EnumProperty("Point type", "None", "the point type of trajectory", parent, SLOT(UpdateScatter()), this);
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

  scatter_color_ = new ColorProperty(
      "scatter color", QColor(Qt::blue), "set the color of all scatter", parent, SLOT(UpdateScatter()), this);

  scatter_size_ = new IntProperty("scatter size", 1, "the size of all scatter", parent, SLOT(UpdateScatter()), this);
  scatter_size_->setMin(1);
  scatter_size_->setMax(10);
  // line
  line_type_ = new EnumProperty("line type", "Line", "the line type of trajectory", parent, SLOT(UpdateLine()), this);
  line_type_->addOption("None", 0);
  line_type_->addOption("Line", 1);

  line_color_ =
      new ColorProperty("line color", QColor(Qt::gray), "set the color of line", parent, SLOT(UpdateLine()), this);
  line_width_ = new IntProperty("line width", 1, "the line width of trajectory", parent, SLOT(UpdateLine()), this);
  line_width_->setMin(1);
  line_width_->setMax(10);
}

void PlotableProperty::UpdateScatter() {
  auto const shape = static_cast<QCPScatterStyle::ScatterShape>(scatter_type_->getOptionInt());
  auto const color = scatter_color_->getColor();
  double const size = scatter_size_->getInt();
  ss_ = QCPScatterStyle{shape, color, size};
  Q_EMIT ScatterTypeChanged(ss_);
}
void PlotableProperty::UpdateLine() {
  int const type = line_type_->getOptionInt();
  line_width_->setHidden(type == 0 ? true : false);
  line_color_->setHidden(type == 0 ? true : false);
  QPen line_pen;
  line_pen.setColor(line_color_->getColor());
  line_pen.setWidth(line_width_->getInt());
  ls_ = LineStyle{type, line_pen};
  Q_EMIT LineTypeChanged(ls_);
}

} // namespace properties
} // namespace rviz_common
