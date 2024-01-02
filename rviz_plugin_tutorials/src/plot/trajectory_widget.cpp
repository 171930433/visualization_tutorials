#include <QWidget>
#include <Eigen/Dense>

#include "plot/trajectory_widget.h"

// #include "zlam_common/structs/inner_types.hpp"
// #include "zlam_common/util/earth.hpp"

// #include "cacher/cacher.hpp"
// #include "cyber_message_filter_display.h"

DisplaySyncBase *TrajectoryWidget::getDisplaySync()
{
  return sync_display_;
}

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

  // addRandomGraph();
  // addRandomGraph();
  // addRandomGraph();
  addRandomTrajectory();
  addRandomTrajectory();
  // all_curve_["gnss"] = new QCPCurve(this->xAxis, this->yAxis);
  // all_curve_["gnss"]->setPen(QPen(QColor(255, 110, 40)));
  // all_curve_["gnss"]->setScatterStyle(QCPScatterStyle::ScatterShape::ssCross);
  // all_curve_["gnss"]->setName("gnss");
  // all_curve_["gnss"]->setSelectable(QCP::stDataRange);

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
  connect(this, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged()));
  // connect(this, SIGNAL(selectionChanged()), this, SLOT(selectionChanged()));

  connect(this, SIGNAL(mouseWheel(QWheelEvent *)), this, SLOT(mouseWheel(QWheelEvent *)));

  // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
  connect(&dataTimer_, SIGNAL(timeout()), this, SLOT(SyncData()));
  dataTimer_.start(100); // Interval 0 means to refresh as fast as possible

  // connect slot that shows a message in the status bar when a graph is clicked:
  connect(this, SIGNAL(plottableClick(QCPAbstractPlottable *, int, QMouseEvent *)), this, SLOT(graphClicked(QCPAbstractPlottable *, int)));
}

void TrajectoryWidget::addRandomTrajectory()
{
  int n = 1 * 1000; // number of points in graph
  double xScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double yScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double xOffset = (std::rand() / (double)RAND_MAX - 0.5) * 4;
  double yOffset = (std::rand() / (double)RAND_MAX - 0.5) * 10;
  double r1 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r2 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r3 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r4 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  QVector<double> x(n), y(n), time_index(n);
  for (int i = 0; i < n; i++)
  {
    time_index[i] = i + 1e8;
    x[i] = (i / (double)n - 0.5) * 10.0 * xScale + xOffset;
    y[i] = (qSin(x[i] * r1 * 5) * qSin(qCos(x[i] * r2) * r4 * 3) + r3 * qCos(qSin(x[i]) * r4 * 2)) * yScale + yOffset;
  }

  QCPCurve *frame = new QCPCurve(this->xAxis, this->yAxis); // 自动注册到graph里面
  frame->setName(QString("curve-%1").arg(all_curve_.size()));
  frame->setScatterStyle(QCPScatterStyle::ScatterShape::ssCross);
  frame->setLineStyle(QCPCurve::LineStyle::lsLine);
  frame->setSelectable(QCP::stDataRange);
  frame->setData(time_index, x, y);
  QPen graphPen;
  graphPen.setColor(QColor(0, 0, 0));
  graphPen.setWidthF(std::rand() / (double)RAND_MAX * 2 + 1);
  frame->setPen(graphPen);

  all_curve_[frame->name().toStdString()] = frame;

  this->replot();
}

void TrajectoryWidget::addRandomGraph()
{
  int n = 1 * 1000; // number of points in graph
  double xScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double yScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double xOffset = (std::rand() / (double)RAND_MAX - 0.5) * 4;
  double yOffset = (std::rand() / (double)RAND_MAX - 0.5) * 10;
  double r1 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r2 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r3 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r4 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  QVector<double> x(n), y(n), time_index(n);
  for (int i = 0; i < n; i++)
  {
    time_index[i] = i;
    x[i] = (i / (double)n - 0.5) * 10.0 * xScale + xOffset;
    y[i] = (qSin(x[i] * r1 * 5) * qSin(qCos(x[i] * r2) * r4 * 3) + r3 * qCos(qSin(x[i]) * r4 * 2)) * yScale + yOffset;
  }

  this->addGraph();
  this->graph()->setSelectable(QCP::stDataRange);
  this->graph()->setName(QString("New graph %1").arg(this->graphCount() - 1));
  // this->graph()->setData(x, y);
  this->graph()->setData(time_index, y);
  // this->graph()->setLineStyle((QCPGraph::LineStyle)(std::rand() % 5 + 1));
  this->graph()->setLineStyle((QCPGraph::LineStyle::lsLine));
  // if (std::rand() % 100 > 50)
  // this->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape)(std::rand() % 14 + 1)));
  this->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape::ssCross)));
  QPen graphPen;
  // graphPen.setColor(QColor(std::rand() % 245 + 10, std::rand() % 245 + 10, std::rand() % 245 + 10));
  graphPen.setColor(QColor(0, 0, 0));
  graphPen.setWidthF(std::rand() / (double)RAND_MAX * 2 + 1);
  this->graph()->setPen(graphPen);
  this->replot();
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

void TrajectoryWidget::SyncData()
{
  // if (!this->isVisible()) {
  //   return;
  // }

  // if (!rviz::g_map_origin_) {
  //   return;
  // }

  // auto const origin = *rviz::g_map_origin_;
  // auto gnss_data = rviz::g_cacher_->GetFrameWithChannleTimeUpperBound<zhito::zloc::ZGnss>("/zhito/h2pu/ublox", current_t0_s_);

  // static size_t index = 0;
  // for (auto const &[key, frame] : gnss_data) {
  //   Eigen::Vector3d pos = zhito::zloc::Earth::DeltaPosEnuInSecondPoint(frame->pos_, origin);
  //   all_curve_["gnss"]->addData(key, pos.x(), pos.y());
  //   current_t0_s_ = key / 1000.0;
  //   // qDebug() << index++ << " time = " << key << " ,state = " << frame->status_ << " x = " << pos.x() << " y = " << pos.y();
  // }

  // this->replot();
}

void TrajectoryWidget::graphClicked(QCPAbstractPlottable *plottable, int dataIndex)
{
  // since we know we only have QCPGraphs in the plot, we can immediately access interface1D()
  // usually it's better to first check whether interface1D() returns non-zero, and only then use it.
  double t0_s = plottable->interface1D()->dataSortKey(dataIndex);
  double x = plottable->interface1D()->dataMainKey(dataIndex);
  double y = plottable->interface1D()->dataMainValue(dataIndex);

  QString message = QString("Clicked on graph '%1' at data point #%2, t0=%5 x= %3 y = %4.").arg(plottable->name()).arg(dataIndex).arg(x).arg(y).arg(t0_s, 0, 'f', 3);
  qDebug() << " " << message;

  FocusPoint(t0_s);
}

void TrajectoryWidget::selectionChanged()
{
  // synchronize selection of graphs with selection of corresponding legend items:
  for (int i = 0; i < this->graphCount(); ++i)
  {
    QCPGraph *graph = this->graph(i);
    QCPPlottableLegendItem *item = this->legend->itemWithPlottable(graph);
    if (item->selected() || graph->selected())
    {
      item->setSelected(true);
      //      graph->setSelection(QCPDataSelection(graph->data()->dataRange()));
    }
  }

  auto *first_curve = all_curve_.begin()->second;
  for (auto [key, curve] : all_curve_)
  {
    auto const &range = curve->selection().dataRange();
    if (!range.isEmpty())
    {
      double const start = curve->dataSortKey(range.begin());
      double const end = curve->dataSortKey(range.end());
      QString str = QString("%1 selected %2 points, t0_s in range [%3,%4]").arg(curve->name()).arg(curve->selection().dataPointCount()).arg(start, 0, 'f', 3).arg(end, 0, 'f', 3);
      qDebug() << str;

      

      if (curve == first_curve)
      {
        FouseRange(QCPRange{start, end});
      }
    }
  }

  qDebug() << " selectionChanged ";
}

void TrajectoryWidget::keyPressEvent(QKeyEvent *event)
{
  if (event->key() == Qt::Key_S)
  {
    if (this->selectionRectMode() == QCP::SelectionRectMode::srmSelect)
    {
      this->setSelectionRectMode(QCP::SelectionRectMode::srmNone);
      QWidget::setCursor(Qt::ArrowCursor);
    }
    else
    {
      this->setSelectionRectMode(QCP::SelectionRectMode::srmSelect);
      QWidget::setCursor(Qt::CrossCursor);
    }
  }
}

void TrajectoryWidget::resizeEvent(QResizeEvent *event)
{
  PlotBase::resizeEvent(event);
  this->replot();
}

void TrajectoryWidget::ChangeScatterShape(QCPScatterStyle::ScatterShape const type)
{
  this->graph()->setScatterStyle(type);
  this->replot();
  qDebug() << "scatter type = " << type << " changed !";
}

void TrajectoryWidget::ChangeLineStyle(QCPGraph::LineStyle const type)
{
  this->graph()->setLineStyle(type);
  this->replot();
  qDebug() << "line type = " << type << " changed !";
}

void TrajectoryWidget::FocusPoint(double const t0)
{
  // 1. 先检查所有的数据区间是否包含待查找点
  auto *first_curve = all_curve_.begin()->second;
  auto const dataIndex = first_curve->findBegin(t0, true)+1;

  double const t0_s = first_curve->dataSortKey(dataIndex);
  double const x = first_curve->dataMainKey(dataIndex);
  double const y = first_curve->dataMainValue(dataIndex);

  QString const str = QString("t0 = %1, finded to_s = %2").arg(t0, 0, 'f', 3).arg(t0_s, 0, 'f', 3);
  qDebug() << str;

  this->xAxis->setRange(x, xAxis->range().size(), Qt::AlignCenter);
  this->yAxis->setRange(y, yAxis->range().size(), Qt::AlignCenter);

  // 选中点
  QCPDataRange index_range{dataIndex, dataIndex + 1};
  first_curve->setSelection(QCPDataSelection{index_range});

  this->replot();
}

void TrajectoryWidget::FouseRange(QCPRange const &time_range)
{
  // 1. 先检查所有的数据区间是否包含待查找区间
  double const t0_s = time_range.center();
  FocusPoint(t0_s);
  // 2. 选中待选择点
  auto *first_curve = all_curve_.begin()->second;
  auto const start_Index = first_curve->findBegin(time_range.lower);
  auto const end_Index = first_curve->findBegin(time_range.upper);
  // 3. 清空原先选中的点
  auto const selected = first_curve->selection();
  QString str1 = QString("origin [%1,%2]").arg(selected.dataRange().begin()).arg(selected.dataRange().end());
  qDebug() << str1;
  QCPDataRange index_range{start_Index, end_Index};
  first_curve->setSelection(QCPDataSelection{index_range});
  QString str2 = QString("after [%1,%2]").arg(start_Index).arg(end_Index);
  qDebug() << str2;
}
