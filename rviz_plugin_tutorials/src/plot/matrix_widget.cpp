#include <QWidget>

#include "plot/matrix_widget.h"
#include "protobuf_helper.h"

MatrixWidget::MatrixWidget(QWidget *parent) : PlotBase(parent)
{
  type_ = Type::Matrix;

  setupMatrixDemo(0, 0);
  connect(this, SIGNAL(mouseWheel(QWheelEvent *)), this, SLOT(mouseWheel()));
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

void MatrixWidget::CreatePlot(QString const &name, MatrixXQString const &field_names)
{
  //
  int const row = field_names.rows(), col = field_names.cols();
  qDebug() << QString("start row=%1 col=%2").arg(row).arg(col);
  setupMatrixDemo(row, col);
  qDebug() << QString("end row=%1 col=%2").arg(row).arg(col);

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
  curve->setSelectable(QCP::stDataRange);

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
  //! 因为0,0 --> 1,1 是行和列单独变化的,
  int const row = (this->plotLayout()->rowCount() < 1 ? 1 : this->plotLayout()->rowCount());
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
  if (new_row < 1 || new_col < 1)
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
      //
      // x轴不显示
      if (i != new_row - 1)
      {
        current_rect->axis(QCPAxis::atBottom)->setTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setSubTicks(false);
        current_rect->axis(QCPAxis::atBottom)->setTickLabels(false);
      }
      current_rect->setAutoMargins(QCP::MarginSide::msAll);
      current_rect->setMinimumMargins(QMargins{0, 0, 0, 0});
    }
  }

  this->plotLayout()->simplify();
  this->replot();
}
