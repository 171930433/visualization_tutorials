#include <QWidget>

#include "plot/plot_base.h"

template <typename _T>
void FoucuPositionByIndex(_T *single_graph, int const dataIndex)
{
  double const x = single_graph->dataMainKey(dataIndex);
  double const y = single_graph->dataMainValue(dataIndex);

  QCPAxis *x_axis = single_graph->keyAxis();
  QCPAxis *y_axis = single_graph->valueAxis();

  x_axis->setRange(x, x_axis->range().size(), Qt::AlignCenter);
  y_axis->setRange(y, y_axis->range().size(), Qt::AlignCenter);
}

template <typename _T>
void FocusPointByT0(_T *single_graph, double const t0)
{
  auto const dataIndex = single_graph->findBegin(t0, true);
  // 选中点
  QCPDataRange index_range{dataIndex, dataIndex + 1};
  single_graph->setSelection(QCPDataSelection{index_range});
  // 点居中
  FoucuPositionByIndex(single_graph, dataIndex);
}

PlotBase::PlotBase(QWidget *parent) : QCustomPlot(parent)
{
  //
  this->setVisible(false);

  connect(this, SIGNAL(selectionChangedByUser()), this, SLOT(onSelectionChangedByUser()));
}

void PlotBase::keyPressEvent(QKeyEvent *event)
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
  QWidget::keyPressEvent(event);
}

void PlotBase::FouseRange(QCPRange const &time_range)
{
}

void PlotBase::FocusPoint(double const t0)
{
  // 第一个矩形
  QCPAxisRect *first_rect = this->axisRect(0);
  QList<QCPAbstractPlottable *> plottables = first_rect->plottables();

  // 第一个graph
  for (auto *single_graph : first_rect->graphs())
  {
    FocusPointByT0(single_graph, t0);
    break;
  }
  // 第一个curve
  for (auto *single_plot : plottables)
  {
    QCPCurve *curve = dynamic_cast<QCPCurve *>(single_plot);
    if (curve)
    {
      FocusPointByT0(curve, t0);
      break;
    }
  }
  this->replot();
}

void PlotBase::onSelectionChangedByUser()
{

  qDebug() << "PlotBase onSelectionChangedByUser begin";

  // 第一个矩形
  QCPAxisRect *first_rect = this->axisRect(0);
  QList<QCPAbstractPlottable *> plottables = first_rect->plottables();

  //
  QCPDataRange range;
  double start_t0 = 0;
  // 第一个graph
  for (auto *single_graph : first_rect->graphs())
  {
    range = single_graph->selection().dataRange();
    if (!range.isEmpty())
    {
      start_t0 = single_graph->dataSortKey(range.begin());
      FoucuPositionByIndex(single_graph, range.begin());
    }
    break;
  }
  // 第一个curve
  for (auto *single_plot : plottables)
  {
    QCPCurve *single_graph = dynamic_cast<QCPCurve *>(single_plot);
    if (single_graph)
    {
      range = single_graph->selection().dataRange();
      if (!range.isEmpty())
      {
        start_t0 = single_graph->dataSortKey(range.begin());
        FoucuPositionByIndex(single_graph, range.begin());
      }
      break;
    }
  }

  if (range.size() == 1)
  {
    this->onFocusPoint(start_t0, false, true);
  }
  else if (range.size() > 1)
  {
  }

  this->replot();
  qDebug() << "PlotBase onSelectionChangedByUser end";
}