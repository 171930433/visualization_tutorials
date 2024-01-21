#include <QWidget>
#include <Eigen/Dense>

#include "plot/trajectory_widget.h"

// #include "zlam_common/structs/inner_types.hpp"
// #include "zlam_common/util/earth.hpp"

// #include "cacher/cacher.hpp"
// #include "cyber_message_filter_display.h"

TrajectoryWidget::TrajectoryWidget(QWidget *parent) : PlotBase(parent)
{
  type_ = Type::Trajectory;
  //
  setupTrajectoryDemo();
}

void TrajectoryWidget::setupTrajectoryDemo()
{
  // demoName = "Vector3 Demo";
  this->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iMultiSelect | QCP::iSelectLegend | QCP::iSelectPlottables);

  this->setMultiSelectModifier(Qt::KeyboardModifier::ControlModifier);
  // this->setSelectionRectMode(QCP::SelectionRectMode::srmSelect);

  this->xAxis->setRange(-8, 8);
  this->yAxis->setRange(-5, 5);
  this->axisRect()->setupFullAxesBox();
  this->axisRect()->setAutoMargins(QCP::MarginSide::msNone);
  this->axisRect()->setMargins(QMargins{0, 0, 0, 0});
  // xy with same scale strategy
  map_ticker_ = QSharedPointer<QCPMapAxisTickerFixed>::create(this->xAxis, this->yAxis);
  map_ticker_->setTickStep(10.0);
  map_ticker_->setScaleStrategy(QCPAxisTickerFixed::ssNone);

  foreach (QCPAxis *axis, this->axisRect()->axes())
  {
    axis->setTicker(map_ticker_);
    axis->setTickLength(0, 0);
    axis->setTickLabels(false);
  }

  this->legend->setVisible(true);
  QFont legendFont = font();
  legendFont.setPointSize(10);
  this->legend->setFont(legendFont);
  this->legend->setSelectedFont(legendFont);
  this->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items

  // xy axis with same scale factor
  this->yAxis->setScaleRatio(this->xAxis, 1.0);

  this->rescaleAxes();

  // 添加分辨率部分
  step_text = new QCPItemText(this);
  step_text->position->setType(QCPItemPosition::ptAxisRectRatio);
  step_text->setPositionAlignment(Qt::AlignRight | Qt::AlignBottom);
  step_text->position->setCoords(1.0, 0.95); // lower right corner of axis rect
  step_text->setText("1 m");
  step_text->setTextAlignment(Qt::AlignLeft);
  step_text->setFont(QFont(font().family(), 9));
  step_text->setPadding(QMargins(8, 0, 0, 0));

  // connect slot that shows a message in the status bar when a graph is clicked:
  // connect(this, SIGNAL(plottableClick(QCPAbstractPlottable *, int, QMouseEvent *)), this, SLOT(graphClicked(QCPAbstractPlottable *,
  // int)));

  // setup policy and connect slot for context menu popup:
  // this->setContextMenuPolicy(Qt::CustomContextMenu);
  // connect(this, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));

  // connect slot that ties some axis selections together (especially opposite axes):
  // connect(this, SIGNAL(selectionChangedByUser()), this, SLOT(onSelectionChangedByUser()));
  // connect(this, SIGNAL(onSelectionChangedByUser()), this, SLOT(onSelectionChangedByUser()));

  connect(this, SIGNAL(mouseWheel(QWheelEvent *)), this, SLOT(mouseWheel(QWheelEvent *)));

}

void TrajectoryWidget::RemoveCurve(QCPCurve *curve)
{
  if (!curve)
  {
    return;
  }
  for (auto it = all_curve_.begin(); it != all_curve_.end(); ++it)
  {
    if (*it == curve)
    {
      all_curve_.erase(it);
      break;
    }
  }
  this->removePlottable(curve);
  curve = nullptr;
}

QCPCurve *TrajectoryWidget::ContainsCurve(QString const &name)
{
  QCPCurve *result = nullptr;
  for (auto it = all_curve_.begin(); it != all_curve_.end(); ++it)
  {
    if ((*it)->name() == name)
    {
      result = *it;
      break;
    }
  }
  return result;
}

QCPCurve *TrajectoryWidget::addTrajectory(QString const &name,                      // curve legend
                                          std::map<size_t, spMessage> const &datas, // 数据消息
                                          QCPScatterStyle const &ss,                // 散点样式
                                          QPen const &lp                            // line pen
)
{
  static double y_offset = 0;

  // raw data
  //
  int const n = datas.size();
  QVector<double> x(n), y(n), time_index(n);
  QStringList const header = GetFildNames(*datas.begin()->second);
  int i = 0;
  for (auto const &kv : datas)
  {
    auto const &message = *kv.second;
    time_index[i] = kv.first / 1e3;
    x[i] = GetValueByHeaderName(message, QString("pos-x")).toDouble();
    y[i] = GetValueByHeaderName(message, QString("pos-y")).toDouble() + y_offset;
    ++i;
  }

  // raw_data_->set(vec_data, true);
  qDebug() << QString("raw_data_ size = %1").arg(datas.size());

  QCPCurve *curve = new QCPCurve(this->xAxis, this->yAxis); // 自动注册到graph里面
  curve->setName(name);
  curve->setSelectable(QCP::stDataRange);
  curve->setData(time_index, x, y);

  // 设置散点样式和颜色
  curve->setScatterStyle(ss);

  // 设置直线样式
  curve->setLineStyle(QCPCurve::LineStyle::lsLine);
  curve->setPen(lp);

  // 定制选中样式
  QCPSelectionDecorator *decorator = curve->selectionDecorator();
  QCPScatterStyle selectedScatterStyle = decorator->scatterStyle();
  selectedScatterStyle.setSize(10);                                                           // 选中点的大小
  decorator->setScatterStyle(selectedScatterStyle, QCPScatterStyle::ScatterProperty::spSize); // 只有size使用设定值，其他的用plot的继承值

  all_curve_.push_back(curve);

  y_offset++;

  return curve;
}

QString TrajectoryWidget::StepString(double const step) // 分辨率文字
{
  QString label = "";

  if (step < 0.01)
    label = QString("%1 mm").arg(step * 1000.0, 0, 'f', 0);
  else if (step < 1.0)
    label = QString("%1 cm").arg(step * 100.0, 0, 'f', 0);
  else if (step < 1000.0)
    label = QString("%1 m").arg(step, 0, 'f', 0);
  else
    label = QString("%1 km").arg(step / 1000.0, 0, 'f', 0);
  return label;
}

void TrajectoryWidget::mouseWheel(QWheelEvent *event)
{
  double const step = map_ticker_->Step();

  // if (step < 1e-3 || step >= 2e4) {
  //   event->accept();
  // }

  step_text->setText(StepString(step));

  // qDebug() << " step = " << step;
}


void TrajectoryWidget::resizeEvent(QResizeEvent *event)
{
  PlotBase::resizeEvent(event);
  this->replot();
}

// void TrajectoryWidget::FouseRange(QCPRange const &time_range)
// {

//   // qDebug() << " TrajectoryWidget::FouseRange called";

//   auto *first_curve = all_curve_.front();

//   for (auto curve : all_curve_)
//   {
//     // 1. 先检查所有的数据区间是否包含待查找点
//     auto const si = curve->findBegin(time_range.lower, true) + 1; // start index
//     auto const ei = curve->findBegin(time_range.upper, true) + 1; // end index

//     // double const t0_s_s = curve->dataSortKey(si);
//     // double const t0_s_e = curve->dataSortKey(ei);
//     // double const x = curve->dataMainKey(si);
//     // double const y = curve->dataMainValue(si);

//     // QString const str = QString("t0 = %1, finded range = [%2,%3]").arg(t0_s_s, 0, 'f', 3).arg(t0_s_s, 0, 'f', 3).arg(t0_s_e, 0, 'f', 3);
//     // qDebug() << str;

//     // 选中点
//     QCPDataRange index_range{si, ei + 1};
//     curve->setSelection(QCPDataSelection{index_range});
//     // 起点剧中
//     if (curve == first_curve)
//     {
//       if (focus_when_select_)
//       {
//         FoucuPositionByIndex(curve, si);
//       }
//     }
//   }

//   // 当前选中点以改变消息发出
//   // this->onFocusPoint(t0, false, true);

//   this->replot();
// }
