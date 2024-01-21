#include <QWidget>

#include "plot/plot_base.h"

void FocusByIndex(QCPAbstractPlottable *single_graph, int const dataIndex)
{
  double const x = single_graph->interface1D()->dataMainKey(dataIndex);
  double const y = single_graph->interface1D()->dataMainValue(dataIndex);

  QCPAxis *x_axis = single_graph->keyAxis();
  QCPAxis *y_axis = single_graph->valueAxis();

  x_axis->setRange(x, x_axis->range().size(), Qt::AlignCenter);
  y_axis->setRange(y, y_axis->range().size(), Qt::AlignCenter);
}

void SelectByT0(QCPAbstractPlottable *single_graph, double const t0)
{
  auto const dataIndex = single_graph->interface1D()->findBegin(t0, true);
  // 选中点
  QCPDataRange index_range{dataIndex, dataIndex + 1};
  single_graph->setSelection(QCPDataSelection{index_range});
}

PlotBase::PlotBase(QWidget *parent) : QCustomPlot(parent)
{
  //
  this->setVisible(false);

  // connect(this, SIGNAL(selectionChangedByUser()), this, SLOT(onSelectionChangedByUser()));
  // connect slot that shows a message in the status bar when a graph is clicked:
  // connect(this, SIGNAL(plottableClick(QCPAbstractPlottable *, int, QMouseEvent *)), this, SLOT(graphClicked(QCPAbstractPlottable *, int)));
  connect(this, &QCustomPlot::plottableClick, this, &PlotBase::graphClicked);
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
    SelectByT0(single_graph, t0);
    break;
  }
  // 第一个curve
  for (auto *single_plot : plottables)
  {
    QCPCurve *curve = dynamic_cast<QCPCurve *>(single_plot);
    if (curve)
    {
      SelectByT0(curve, t0);
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
      // FocusByIndex(single_graph, range.begin());
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
        // FocusByIndex(single_graph, range.begin());
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

void PlotBase::graphClicked(QCPAbstractPlottable *plottable, int dataIndex)
{
  // since we know we only have QCPGraphs in the plot, we can immediately access interface1D()
  // usually it's better to first check whether interface1D() returns non-zero, and only then use it.
  double t0_s = plottable->interface1D()->dataSortKey(dataIndex);
  double x = plottable->interface1D()->dataMainKey(dataIndex);
  double y = plottable->interface1D()->dataMainValue(dataIndex);

  // 被点击的序列居中
  FocusByIndex(plottable, dataIndex);

  // 1. 当前rect,其余序列被选中
  // 2. 其余rect, 主序列居中,其余序列被选中

  // 当前矩形
  QCPAxisRect *current_rect = plottable->keyAxis()->axisRect();
  for (auto *single_rect : this->axisRects())
  {
    // 当前rect
    if (single_rect == current_rect)
    {
      for (auto *single_plotable : single_rect->plottables())
      {
        if (single_plotable != plottable)
        {
          SelectByT0(single_plotable, t0_s);
        }
      }
    }
    // 其余rect
    else
    {
      for (auto *single_plotable : single_rect->plottables())
      {
        SelectByT0(single_plotable, t0_s);
      }
    }
  }

  // QList<QCPAbstractPlottable *> plottables = first_rect->plottables();

  QString message = QString("Clicked on graph '%1' at data point #%2, t0=%5 x= %3 y = %4.").arg(plottable->name()).arg(dataIndex).arg(x).arg(y).arg(t0_s, 0, 'f', 3);
  qDebug() << " " << message;

  this->onFocusPoint(t0_s, false, true);
}