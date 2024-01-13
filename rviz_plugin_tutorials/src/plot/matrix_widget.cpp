#include <QWidget>

#include "plot/matrix_widget.h"
#include "protobuf_helper.h"

MatrixWidget::MatrixWidget(QWidget *parent) : PlotBase(parent)
{
  type_ = Type::Matrix;
  dateTicker_ = QSharedPointer<QCPAxisTickerDateTime>(new QCPAxisTickerDateTime);
  dateTicker_->setDateTimeFormat("HH:mm:ss");
  this->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes);
  connect(this, SIGNAL(mouseWheel(QWheelEvent *)), this, SLOT(mouseWheel()));
  // setupMatrixDemo(1, 1);
  // setupVector3Demo();
  // 默认状态
  this->plotLayout()->clear(); // let's start from scratch and remove the default axis rect
  auto *rect = CreateDefaultRect();
  this->plotLayout()->addElement(0, 0, rect);
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

  for (auto rect : this->axisRects())
  {
    if (rect->axis(QCPAxis::atBottom)->selectedParts().testFlag(QCPAxis::spAxis))
    {
      x_selected = true;
      break;
    }
  }

  for (auto rect : this->axisRects())
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

  for (auto *rect : this->axisRects())
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
    if (!single_graph->dataCount())
    {
      continue;
    }

    auto const dataIndex = single_graph->findBegin(t0, true);

    // 选中点
    QCPDataRange index_range{dataIndex, dataIndex + 1};
    single_graph->setSelection(QCPDataSelection{index_range});
    // 该点剧中
    FoucuPositionByIndex(single_graph, dataIndex);
  }

  this->replot();
}

void MatrixWidget::CreatePlot(QString const &name, MatrixXQString const &field_names)
{
  //
  int const row = field_names.rows(), col = field_names.cols();
  qDebug() << QString("start row=%1 col=%2").arg(row).arg(col);
  setupMatrixDemo(row, col);

  // this->rescaleAxes();
  for (int i = 0; i < row; i++)
  {
    for (int j = 0; j < col; j++)
    {
      UpdateFieldName(i, j, field_names(i, j));
    }
  }

  qDebug() << QString("end filed size =%1").arg(field_names.size());
}

void MatrixWidget::keyPressEvent(QKeyEvent *event)
{
  ShowSubplot(event->key() - Qt::Key_1);
  QWidget::keyPressEvent(event);
}

void MatrixWidget::UpdateFieldName(int const row, int const col, QString const &field_name)
{
  qDebug() << QString("UpdateFieldName start %1, row=%2, col=%3").arg(field_name).arg(row).arg(col);
  // qDebug() << " rects_.size() = " << rects_.size() << " g_messages size = " << g_messages.size();
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

  qDebug() << " time_index = size = " << time_index.size() << " y size =" << y.size();

  auto *rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(row, col));

  QCPGraph *curve = rect->graphs()[0];

  curve->setData(time_index, y);
  rect->axis(QCPAxis::atLeft)->setLabel(field_name);
  curve->setName(field_name);
  this->rescaleAxes();
  this->replot();

  qDebug() << QString("UpdateFieldName end %1, size = %2").arg(field_name).arg(n);
}

void MatrixWidget::RowChanged(int const new_row)
{
  int const old_row = this->plotLayout()->rowCount();
  int const col = this->plotLayout()->columnCount();
  qDebug() << QString("begin RowChanged, col=%3, %1--->%2").arg(old_row).arg(new_row).arg(col);

  // 增加
  if (old_row < new_row)
  {
    // rects_.conservativeResize(new_row, col);
    for (int i = old_row; i < new_row; i++)
    {
      for (int j = 0; j < col; j++)
      {
        QCPAxisRect *current_rect = CreateDefaultRect();
        bool re = this->plotLayout()->addElement(i, j, current_rect);
      }
    }
  }
  // 减少
  else if (old_row > new_row)
  {
    for (int i = new_row; i < old_row; i++)
    {
      for (int j = 0; j < col; j++)
      {
        QCPAxisRect *current_rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(i, j));
        for (auto *one_graph : current_rect->graphs())
        {
          this->removeGraph(one_graph);
        }
        this->plotLayout()->remove(current_rect);
      }
    }
    // rects_.conservativeResize(new_row, col);
  }
  qDebug() << QString("end RowChanged %1--->%2").arg(old_row).arg(new_row);
}
void MatrixWidget::ColChanged(int const new_col)
{
  int const row = this->plotLayout()->rowCount();
  int const old_col = this->plotLayout()->columnCount();
  qDebug() << QString("begin ColChanged, row=%3, %1--->%2").arg(old_col).arg(new_col).arg(row);

  // 增加
  if (old_col < new_col)
  {
    // rects_.conservativeResize(row, new_col);
    // qDebug() << QString("rects_ %1*%2").arg(this->plotLayout()->rowCount()).arg(this->plotLayout()->columnCount());
    for (int i = 0; i < row; i++)
    {
      for (int j = old_col; j < new_col; j++)
      {
        QCPAxisRect *current_rect = CreateDefaultRect();
        bool re = this->plotLayout()->addElement(i, j, current_rect);
      }
    }
  }
  // 减少
  else if (old_col > new_col)
  {
    for (int i = 0; i < row; i++)
    {
      for (int j = new_col; j < old_col; j++)
      {
        QCPAxisRect *current_rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(i, j));
        for (auto *one_graph : current_rect->graphs())
        {
          this->removeGraph(one_graph);
        }
        this->plotLayout()->remove(current_rect);
      }
    }
    // rects_.conservativeResize(row, new_col);
  }
  qDebug() << QString("end ColChanged %1--->%2").arg(old_col).arg(new_col);
}

void MatrixWidget::UpdatePlotLayout(int const new_row, int const new_col)
{
  int const old_row = this->plotLayout()->rowCount();
  int const old_col = this->plotLayout()->columnCount();
  if (new_row <= 1 && new_col <= 1 && old_row == 1 && old_col == 1)
  {
    return;
  }
  // int const old_row = this->plotLayout()->rowCount();
  // int const old_col = rects_.cols();
  if (new_row != old_row)
  {
    RowChanged(new_row);
  }
  if (old_col != new_col)
  {
    ColChanged(new_col);
  }

  qDebug() << QString("from %1*%2 ------------> %3*%4 rects_ size=%5*%6").arg(old_row).arg(old_col).arg(new_row).arg(new_col).arg(this->plotLayout()->rowCount()).arg(this->plotLayout()->columnCount());
  for (int i = 0; i < new_row; i++)
  {
    for (int j = 0; j < new_col; j++)
    {
      // QCPAxisRect *current_rect = rects_(i, j);
      QCPAxisRect *current_rect = qobject_cast<QCPAxisRect *>(this->plotLayout()->element(i, j));
      // x轴不显示
      if (i != new_row - 1)
      {
        current_rect->axis(QCPAxis::atBottom)->setTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setSubTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setTickLabels(false);
      }
    }
  }
  this->plotLayout()->simplify();
  this->replot();
}

void MatrixWidget::setupMatrixDemo(int row, int col)
{
  using namespace Eigen;
  // demoName = "Vector3 Demo";

  // rects_.resize(row, col);

  for (int i = 0; i < row; i++)
  {
    for (int j = 0; j < col; j++)
    {
      QCPAxisRect *current_rect = CreateDefaultRect();
      this->plotLayout()->addElement(i, j, current_rect);
      // x轴不显示
      if (i != row - 1)
      {
        current_rect->axis(QCPAxis::atBottom)->setTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setSubTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setTickLabels(false);
      }
    }
  }
  this->replot();
}

QCPGraph *MatrixWidget::CreateDefaultGraph(QCPAxisRect *rect)
{

  auto *curve = this->addGraph(rect->axis(QCPAxis::atBottom), rect->axis(QCPAxis::atLeft));

  curve->setSelectable(QCP::stDataRange);

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

  return curve;
}
QCPAxisRect *MatrixWidget::CreateDefaultRect()
{
  static int rect_count = 0;
  QCPAxisRect *rect = new QCPAxisRect(this);
  rect->axis(QCPAxis::atLeft)->setLabel(QString("rect-%1").arg(rect_count++));
  // 默认缩放y轴
  rect->setRangeZoom(Qt::Vertical);
  // x轴样式
  rect->axis(QCPAxis::atBottom)->setTicker(dateTicker_);
  // y轴label在axis内侧
  rect->axis(QCPAxis::atLeft)->setTickLabelSide(QCPAxis::lsInside);
  rect->axis(QCPAxis::atLeft)->setSubTicks(false);
  // 共x轴 (0,0)->所有 所有->(0,0)
  if (this->graphCount() != 0)
  {
    connect(rect->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), this->axisRect(0)->axis(QCPAxis::atBottom),
            SLOT(setRange(QCPRange)));
    connect(this->axisRect(0)->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), rect->axis(QCPAxis::atBottom),
            SLOT(setRange(QCPRange)));
  }
  foreach (QCPAxis *axis, rect->axes())
  {
    axis->setLayer("axes");
    axis->grid()->setLayer("grid");
  }
  // 创建默认序列
  auto *curve = CreateDefaultGraph(rect);
  return rect;
}