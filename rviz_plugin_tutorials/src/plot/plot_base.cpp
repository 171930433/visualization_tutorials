#include <QWidget>

#include "plot/plot_base.h"

PlotBase::PlotBase(QWidget *parent) : QCustomPlot(parent)
{
  //
  this->setVisible(false);
  connect(this, &QCustomPlot::beforeReplot, this, &PlotBase::onBeforeReplot);
}

void PlotBase::onBeforeReplot()
{
}

// void PlotBase::simplifyData(QCPAbstractPlottable1D *curve, double pixelTolerance)
// {
  

//   // int data_size = curve->dataCount();
//   int data_size = raw_data_->size();
//   qDebug() << QString("simplified %1").arg(data_size);

//   QVector<double> simplifiedT, simplifiedX, simplifiedY;
//   simplifiedT.reserve(data_size);
//   simplifiedX.reserve(data_size);
//   simplifiedY.reserve(data_size);

//   // pixelTolerance 是两个点在屏幕上能被区分的最小像素距离
//   if (data_size <= 0)
//   {
//     return; // 确保数据有效
//   }

//   double lastX = raw_data_->at(0)->mainKey();
//   double lastY = raw_data_->at(0)->mainValue();
//   double const pixel_per_meter = map_ticker_->pixel_per_meter();
//   double const meterTolerance = pixel_per_meter * pixelTolerance;
//   simplifiedT.push_back(raw_data_->at(0)->sortKey());
//   simplifiedX.push_back(lastX);
//   simplifiedY.push_back(lastY);

//   QCPRange const x_range = this->xAxis->range();
//   QCPRange const y_range = this->yAxis->range();

//   for (int i = 1; i < data_size; ++i)
//   {
//     double const xi = raw_data_->at(i)->mainKey();
//     double const yi = raw_data_->at(i)->mainValue();
//     // 可视区域内
//     if (x_range.contains(xi) && y_range.contains(yi))
//     {
//       // 检查当前点与上一个点之间的距离是否足够大
//       if (std::abs(xi - lastX) > meterTolerance || std::abs(yi - lastY) > meterTolerance)
//       {
//         simplifiedT.push_back(raw_data_->at(i)->sortKey());
//         simplifiedX.push_back(raw_data_->at(i)->mainKey());
//         simplifiedY.push_back(raw_data_->at(i)->mainValue());

//         lastX = simplifiedX.back();
//         lastY = simplifiedY.back();
//       }
//     }
//   }

//   qDebug() << QString("simplified %1-->%2").arg(data_size).arg(simplifiedT.size());
//   curve->setData(simplifiedT, simplifiedX, simplifiedY);

// }

// void TrajectoryWidget::FoucuPositionByIndex(QCPCurve *curve, int const dataIndex)
// {
//   double const x = curve->dataMainKey(dataIndex);
//   double const y = curve->dataMainValue(dataIndex);

//   this->xAxis->setRange(x, xAxis->range().size(), Qt::AlignCenter);
//   this->yAxis->setRange(y, yAxis->range().size(), Qt::AlignCenter);
// }