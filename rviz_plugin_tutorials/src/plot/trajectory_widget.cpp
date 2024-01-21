#include <QWidget>
#include <Eigen/Dense>

#include "plot/trajectory_widget.h"


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
  step_text_ = new QCPItemText(this);
  step_text_->position->setType(QCPItemPosition::ptAxisRectRatio);
  step_text_->setPositionAlignment(Qt::AlignRight | Qt::AlignBottom);
  step_text_->position->setCoords(1.0, 0.95); // lower right corner of axis rect
  step_text_->setText("1 m");
  step_text_->setTextAlignment(Qt::AlignLeft);
  step_text_->setFont(QFont(font().family(), 9));
  step_text_->setPadding(QMargins(8, 0, 0, 0));

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
  selectedScatterStyle.setSize(10);              // 选中点的大小
  selectedScatterStyle.setPen(QColor(Qt::blue)); // 选中点的颜色

  decorator->setScatterStyle(selectedScatterStyle, QCPScatterStyle::ScatterProperty::spSize | QCPScatterStyle::ScatterProperty::spPen); // 只有size使用设定值，其他的用plot的继承值

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

  step_text_->setText(StepString(step));
}
