#include <QWidget>

#include "plot/plot_base.h"

PlotBase::PlotBase(QWidget *parent) : QCustomPlot(parent) {
  //
  this->setVisible(false);
}

// void TrajectoryWidget::FoucuPositionByIndex(QCPCurve *curve, int const dataIndex)
// {
//   double const x = curve->dataMainKey(dataIndex);
//   double const y = curve->dataMainValue(dataIndex);

//   this->xAxis->setRange(x, xAxis->range().size(), Qt::AlignCenter);
//   this->yAxis->setRange(y, yAxis->range().size(), Qt::AlignCenter);
// }