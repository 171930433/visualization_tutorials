#include <QWidget>
#include <Eigen/Dense>

#include "plot/precision_widget.h"

PrecisionWidget::PrecisionWidget(QWidget *parent) : PlotBase(parent)
{
  type_ = Type::Precision;
  //
  setupMatrixDemo(3, 1);
}
