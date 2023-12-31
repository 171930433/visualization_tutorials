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

  addRandomGraph();
  // addRandomGraph();
  // addRandomGraph();
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

  connect(this, SIGNAL(mouseWheel(QWheelEvent *)), this, SLOT(mouseWheel(QWheelEvent *)));

  // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
  connect(&dataTimer_, SIGNAL(timeout()), this, SLOT(SyncData()));
  dataTimer_.start(100); // Interval 0 means to refresh as fast as possible

  // connect slot that shows a message in the status bar when a graph is clicked:
  connect(this, SIGNAL(plottableClick(QCPAbstractPlottable *, int, QMouseEvent *)), this, SLOT(graphClicked(QCPAbstractPlottable *, int)));
}

void TrajectoryWidget::addRandomGraph()
{
  int n = 50; // number of points in graph
  double xScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double yScale = (std::rand() / (double)RAND_MAX + 0.5) * 2;
  double xOffset = (std::rand() / (double)RAND_MAX - 0.5) * 4;
  double yOffset = (std::rand() / (double)RAND_MAX - 0.5) * 10;
  double r1 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r2 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r3 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  double r4 = (std::rand() / (double)RAND_MAX - 0.5) * 2;
  QVector<double> x(n), y(n);
  for (int i = 0; i < n; i++)
  {
    x[i] = (i / (double)n - 0.5) * 10.0 * xScale + xOffset;
    y[i] = (qSin(x[i] * r1 * 5) * qSin(qCos(x[i] * r2) * r4 * 3) + r3 * qCos(qSin(x[i]) * r4 * 2)) * yScale + yOffset;
  }

  this->addGraph();
  this->graph()->setSelectable(QCP::stDataRange);
  this->graph()->setName(QString("New graph %1").arg(this->graphCount() - 1));
  this->graph()->setData(x, y);
  this->graph()->setLineStyle((QCPGraph::LineStyle)(std::rand() % 5 + 1));
  if (std::rand() % 100 > 50)
    this->graph()->setScatterStyle(QCPScatterStyle((QCPScatterStyle::ScatterShape)(std::rand() % 14 + 1)));
  QPen graphPen;
  graphPen.setColor(QColor(std::rand() % 245 + 10, std::rand() % 245 + 10, std::rand() % 245 + 10));
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
  double dataValue = plottable->interface1D()->dataMainValue(dataIndex);
  QString message = QString("Clicked on graph '%1' at data point #%2 with value %3.").arg(plottable->name()).arg(dataIndex).arg(dataValue);
  qDebug() << " " << message;
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

  for (auto [key, curve] : all_curve_)
  {
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

void TrajectoryWidget::ChangeScatterShape(QCPScatterStyle::ScatterShape const type)
{
  this->graph()->setScatterStyle(type);
  this->replot();
  qDebug() << "scatter type = " << type <<" changed !";
}

void TrajectoryWidget::ChangeLineStyle(QCPGraph::LineStyle const type)
{
  this->graph()->setLineStyle(type);
  this->replot();
  qDebug() << "line type = " << type <<" changed !";
}
