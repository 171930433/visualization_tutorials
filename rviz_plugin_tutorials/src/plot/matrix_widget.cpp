#include <QWidget>

#include "plot/matrix_widget.h"
#include "protobuf_helper.h"

MatrixWidget::MatrixWidget(QWidget *parent) : PlotBase(parent)
{
  //
  type_ = Type::Matrix;
  this->plotLayout()->clear(); // let's start from scratch and remove the default axis rect
  connect(this, SIGNAL(mouseWheel(QWheelEvent *)), this, SLOT(mouseWheel()));

  // setupVector3Demo();
}

DisplaySyncBase *MatrixWidget::getDisplaySync()
{
  return sync_display_;
}

void MatrixWidget::ShowSubplot(int const index)
{
  auto rect_show = this->plotLayout()->take(all_rects_[index]);
  // qDebug() <<  "this->plotLayout()->rowCount() = " << this->plotLayout()->rowCount();
  if (rect_show)
  {
    all_rects_[index]->setVisible(false);
    this->plotLayout()->simplify();
  }
  else
  {
    all_rects_[index]->setVisible(true);
    int current_index = -1; // 因为每次删除元素后，剩余元素会重新编号，所以需要重新计算当前rect的实际index
    for (int i = 0; i <= index; ++i)
    {
      if (all_rects_[i]->visible())
      {
        current_index++;
      }
    }

    // qDebug() << "current_index == " << current_index;

    this->plotLayout()->insertRow(current_index);
    this->plotLayout()->addElement(current_index, 0, all_rects_[index]);
  }
  // for (auto [key, rect] : all_rects_)
  // {
  //   qDebug() << "rect" << key << " " << rect->visible();
  // }
  // qDebug() <<  "end rowCount() = " << this->plotLayout()->rowCount();

  this->replot();
}

void MatrixWidget::contextMenuRequest(QPoint pos)
{
  QMenu *menu = new QMenu(this);
  menu->setAttribute(Qt::WA_DeleteOnClose);

  menu->addAction("X plot", this, [this]
                  { ShowSubplot(0); });
  menu->addAction("Y plot", this, [this]
                  { ShowSubplot(1); });
  menu->addAction("Z plot", this, [this]
                  { ShowSubplot(2); });

  menu->popup(this->mapToGlobal(pos));
}

void MatrixWidget::mouseWheel()
{
  bool x_selected = false;
  for (auto [key, rect] : all_rects_)
  {
    if (rect->axis(QCPAxis::atBottom)->selectedParts().testFlag(QCPAxis::spAxis))
    {
      x_selected = true;
      break;
    }
  }

  for (auto [key, rect] : all_rects_)
  {
    if (x_selected)
    {
      rect->setRangeZoom(Qt::Horizontal);
    }
    else
    {
      rect->setRangeZoom(Qt::Vertical);
    }
  }
  // qDebug() <<" x_selected = " << x_selected;
}

void MatrixWidget::setupVector3Demo()
{
  // demoName = "Vector3 Demo";
  this->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes);

  QSharedPointer<QCPAxisTickerDateTime> dateTicker(new QCPAxisTickerDateTime);
  // 设置日期时间格式
  dateTicker->setDateTimeFormat("HH:mm:ss");

  this->plotLayout()->clear(); // let's start from scratch and remove the default axis rect
  // add the first axis rect in second row (row index 1):
  QCPAxisRect *topAxisRect = new QCPAxisRect(this);
  QCPAxisRect *middleAxisRect = new QCPAxisRect(this);
  QCPAxisRect *bottomAxisRect = new QCPAxisRect(this);
  all_rects_[0] = topAxisRect;
  all_rects_[1] = middleAxisRect;
  all_rects_[2] = bottomAxisRect;
  this->plotLayout()->addElement(0, 0, topAxisRect);
  this->plotLayout()->addElement(1, 0, middleAxisRect);
  this->plotLayout()->addElement(2, 0, bottomAxisRect);
  // 默认缩放y轴
  topAxisRect->setRangeZoom(Qt::Vertical);
  middleAxisRect->setRangeZoom(Qt::Vertical);
  bottomAxisRect->setRangeZoom(Qt::Vertical);
  // x轴样式
  topAxisRect->axis(QCPAxis::atBottom)->setTicker(dateTicker);
  middleAxisRect->axis(QCPAxis::atBottom)->setTicker(dateTicker);
  bottomAxisRect->axis(QCPAxis::atBottom)->setTicker(dateTicker);

  // x轴的padding为0
  // topAxisRect->axis(QCPAxis::atBottom)->setPadding(0);
  // topAxisRect->setMargins(QMargins{0,0,0,0});
  // middleAxisRect->setMargins(QMargins{0,0,0,0});
  topAxisRect->setAutoMargins(QCP::msLeft | QCP::msRight);
  topAxisRect->setMargins(QMargins(0, 0, 0, 0));
  middleAxisRect->setAutoMargins(QCP::msLeft | QCP::msRight);
  middleAxisRect->setMargins(QMargins(0, 0, 0, 0));
  bottomAxisRect->setAutoMargins(QCP::msLeft | QCP::msRight | QCP::msBottom);
  bottomAxisRect->setMargins(QMargins(0, 0, 0, 0));
  // this->plotLayout()->setRowSpacing(0);
  // this->plotLayout()->setRowStretchFactor(0, 1);
  // 共x轴
  connect(bottomAxisRect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), middleAxisRect->axis(QCPAxis::atBottom),
          SLOT(setRange(QCPRange)));
  connect(middleAxisRect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), topAxisRect->axis(QCPAxis::atBottom),
          SLOT(setRange(QCPRange)));
  connect(topAxisRect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), bottomAxisRect->axis(QCPAxis::atBottom),
          SLOT(setRange(QCPRange)));
  // y轴label在axis内侧
  topAxisRect->axis(QCPAxis::atLeft)->setTickLabelSide(QCPAxis::lsInside);
  middleAxisRect->axis(QCPAxis::atLeft)->setTickLabelSide(QCPAxis::lsInside);
  bottomAxisRect->axis(QCPAxis::atLeft)->setTickLabelSide(QCPAxis::lsInside);
  topAxisRect->axis(QCPAxis::atLeft)->setSubTicks(false);
  middleAxisRect->axis(QCPAxis::atLeft)->setSubTicks(false);
  bottomAxisRect->axis(QCPAxis::atLeft)->setSubTicks(false);
  // top和middle的x轴不显示
  topAxisRect->axis(QCPAxis::atBottom)->setTicks(false);
  topAxisRect->axis(QCPAxis::atBottom)->setSubTicks(false);
  topAxisRect->axis(QCPAxis::atBottom)->setTickLabels(false);
  middleAxisRect->axis(QCPAxis::atBottom)->setTicks(false);
  middleAxisRect->axis(QCPAxis::atBottom)->setSubTicks(false);
  middleAxisRect->axis(QCPAxis::atBottom)->setTickLabels(false);

  QList<QCPAxis *> allAxes;
  allAxes << topAxisRect->axes() << middleAxisRect->axes() << bottomAxisRect->axes();
  foreach (QCPAxis *axis, allAxes)
  {
    axis->setLayer("axes");
    axis->grid()->setLayer("grid");
  }

  // addRandomGraph();
  // this->rescaleAxes();

  // this->setContextMenuPolicy(Qt::CustomContextMenu);
  // connect(this, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));

  connect(this, SIGNAL(mouseWheel(QWheelEvent *)), this, SLOT(mouseWheel()));
}

void MatrixWidget::addRandomGraph()
{
  int n = 500000; // number of points in graph
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

  for (auto [key, rect] : all_rects_)
  {
    QCPAxis *left = rect->axis(QCPAxis::atLeft);
    QCPAxis *bottom = rect->axis(QCPAxis::atBottom);

    this->addGraph(bottom, left);
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

    // 定制选中样式
    QCPSelectionDecorator *decorator = this->graph()->selectionDecorator();
    QCPScatterStyle selectedScatterStyle = decorator->scatterStyle();
    selectedScatterStyle.setSize(10);                                                           // 选中点的大小
    decorator->setScatterStyle(selectedScatterStyle, QCPScatterStyle::ScatterProperty::spSize); // 只有size使用设定值，其他的用plot的继承值
  }

  this->replot();
}

void MatrixWidget::FoucuPositionByIndex(QCPGraph *single_graph, int const dataIndex)
{
  double const x = single_graph->dataMainKey(dataIndex);
  double const y = single_graph->dataMainValue(dataIndex);

  this->xAxis->setRange(x, xAxis->range().size(), Qt::AlignCenter);
  this->yAxis->setRange(y, yAxis->range().size(), Qt::AlignCenter);
}

void MatrixWidget::FocusPoint(double const t0)
{
  // auto *first_curve = all_curve_.front();

  for (int i = 0; i < this->graphCount(); ++i)
  {
    auto *single_graph = this->graph(i);

    // 1. 先检查所有的数据区间是否包含待查找点
    auto const dataIndex = single_graph->findBegin(t0, true) + 1;

    // 选中点
    QCPDataRange index_range{dataIndex, dataIndex + 1};
    single_graph->setSelection(QCPDataSelection{index_range});
    // 该点剧中
    FoucuPositionByIndex(single_graph, dataIndex);
  }

  // 当前选中点以改变消息发出
  // this->onFocusPoint(t0, false, true);

  this->replot();
}

void MatrixWidget::CreatePlot(QString const &name, MatrixXQString const &field_names)
{
  //
  int const row = field_names.rows(), col = field_names.cols();
  qDebug() << QString("start row=%1 col=%2").arg(row).arg(col);
  setupMatrixDemo(row, col);

  for (int i = 0; i < row; i++)
  {
    for (int j = 0; j < col; j++)
    {
      auto *rect = rects_(i, j);
      auto *curve = this->addGraph(rect->axis(QCPAxis::atBottom), rect->axis(QCPAxis::atLeft));

      graphs_(i, j) = curve;

      rect->axis(QCPAxis::atLeft)->setLabel(field_names(i, j));
      curve->setName(field_names(i, j));
      curve->setSelectable(QCP::stDataRange);
      // curve->setData(time_index, y[i]);

      // 设置散点样式和颜色
      curve->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ScatterShape::ssCross, Qt::blue));

      // 设置直线样式
      curve->setLineStyle(QCPGraph::LineStyle::lsLine);
      QPen lp;
      lp.setWidthF(2);
      lp.setColor(Qt::gray);
      curve->setPen(lp);

      // 定制选中样式
      QCPSelectionDecorator *decorator = curve->selectionDecorator();
      QCPScatterStyle selectedScatterStyle = decorator->scatterStyle();
      selectedScatterStyle.setSize(10);                                                           // 选中点的大小
      decorator->setScatterStyle(selectedScatterStyle, QCPScatterStyle::ScatterProperty::spSize); // 只有size使用设定值，其他的用plot的继承值
    }
  }

  // this->rescaleAxes();

  qDebug() << QString("end filed size =%1").arg(field_names.size());
}

void MatrixWidget::keyPressEvent(QKeyEvent *event)
{
  ShowSubplot(event->key() - Qt::Key_1);
}

void MatrixWidget::UpdateFieldName(int const row, int const col, QString const &field_name)
{
  qDebug() << QString("UpdateFieldName start %1, row = %2").arg(field_name).arg(row);
  //
  auto const &datas = g_messages;

  int const n = datas.size();
  int i = 0;
  QVector<double> y(n), time_index(n);
  // QStringList const header = GetFildNames(*datas.begin()->second);
  for (auto const &kv : datas)
  {
    auto const &message = *kv.second;

    time_index[i] = kv.first / 1e3;
    y[i] = GetValueByHeaderName(message, field_name).toDouble();
    ++i;
  }

  graphs_(row, col)->setData(time_index, y);
  this->rescaleAxes();
  this->replot();

  qDebug() << QString("UpdateFieldName end %1, size = %2").arg(field_name).arg(n);
}

void MatrixWidget::UpdatePlotLayout(int const new_row, int const new_col)
{
  int const old_row = rects_.rows();
  int const old_col = rects_.cols();
  
}

void MatrixWidget::setupMatrixDemo(int row, int col)
{
  using namespace Eigen;
  // demoName = "Vector3 Demo";
  this->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes);

  QSharedPointer<QCPAxisTickerDateTime> dateTicker(new QCPAxisTickerDateTime);
  // 设置日期时间格式
  dateTicker->setDateTimeFormat("HH:mm:ss");

  rects_ = Eigen::Matrix<QCPAxisRect *, Eigen::Dynamic, Eigen::Dynamic>(row, col);
  graphs_ = Eigen::Matrix<QCPGraph *, Eigen::Dynamic, Eigen::Dynamic>(row, col);
  for (int i = 0; i < row * col; ++i)
  {
    rects_.data()[i] = new QCPAxisRect(this);
  }

  // rects_.unaryExpr([this](QCPAxisRect *&rect)
  //                  { rect = new QCPAxisRect(this); });

  for (int i = 0; i < row; i++)
  {
    for (int j = 0; j < col; j++)
    {
      QCPAxisRect *rect = new QCPAxisRect(this);
      rects_(i, j) = rect;
      this->plotLayout()->addElement(i, j, rect);
      // 默认缩放y轴
      rect->setRangeZoom(Qt::Vertical);
      // x轴样式
      rect->axis(QCPAxis::atBottom)->setTicker(dateTicker);
      // 共x轴
      if (rect != rects_(0, 0))
      {
        connect(rect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), rects_(0, 0)->axis(QCPAxis::atBottom),
                SLOT(setRange(QCPRange)));
      }
      connect(rects_(0, 0)->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), rect->axis(QCPAxis::atBottom),
              SLOT(setRange(QCPRange)));
      // y轴label在axis内侧
      rect->axis(QCPAxis::atLeft)->setTickLabelSide(QCPAxis::lsInside);
      rect->axis(QCPAxis::atLeft)->setSubTicks(false);
      // top和middle的x轴不显示
      if (i != row - 1)
      {
        rect->axis(QCPAxis::atBottom)->setTicks(false);
        rect->axis(QCPAxis::atBottom)->setSubTicks(false);
        rect->axis(QCPAxis::atBottom)->setTickLabels(false);
      }
      foreach (QCPAxis *axis, rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
  }
}
