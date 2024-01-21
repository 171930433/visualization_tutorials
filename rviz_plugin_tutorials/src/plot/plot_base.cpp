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

// 选中区间
void SelectByT0sT0e(QCPAbstractPlottable *single_graph, double const t0_s, double const t0_e)
{
  auto const dataIndex_s = single_graph->interface1D()->findBegin(t0_s, true);
  auto const dataIndex_e = single_graph->interface1D()->findBegin(t0_e, true);
  // 选中点
  QCPDataRange index_range{dataIndex_s, dataIndex_e + 1};
  single_graph->setSelection(QCPDataSelection{index_range});
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
  for (auto *single_rect : this->axisRects())
  {
    for (auto *single_plotable : single_rect->plottables())
    {
      SelectByT0sT0e(single_plotable, time_range.lower, time_range.upper);
    }
  }

  this->replot();
}

void PlotBase::FocusPoint(double const t0)
{
  for (auto *single_rect : this->axisRects())
  {
    for (auto *single_plotable : single_rect->plottables())
    {
      SelectByT0(single_plotable, t0);
    }
  }

  this->replot();
}

void PlotBase::onSelectionChangedByUser()
{

  qDebug() << "PlotBase onSelectionChangedByUser begin";

  QList<QCPAbstractPlottable *> selected_plotable = this->selectedPlottables();

  if (selected_plotable.size() <= 0)
  {
    qDebug() << " no serials select";
    return;
  }

  if (selected_plotable.size() > 1)
  {
    qDebug() << " select more than one serials";
    return;
  }

  QCPAbstractPlottable *single_plot = selected_plotable.first();
  QCPDataRange index_range = single_plot->selection().dataRange(0);
  double t0_s = single_plot->interface1D()->dataSortKey(index_range.begin());
  double t0_e = single_plot->interface1D()->dataSortKey(index_range.end());

  for (auto *single_rect : this->axisRects())
  {
    for (auto *single_plotable : single_rect->plottables())
    {
      if (index_range.size() == 1)
      {
        SelectByT0(single_plotable, t0_s);
      }
      else if (index_range.size() > 1)
      {
        SelectByT0sT0e(single_plotable, t0_s, t0_e);
      }
    }
  }

  if (index_range.size() == 1)
  {
    this->onFocusPoint(t0_s, false, true);
  }
  else if (index_range.size() > 1)
  {
    QCPRange time_range{t0_s, t0_e};
    this->onFouseRange(time_range, false, true);
  }

  this->replot();
  qDebug() << "PlotBase onSelectionChangedByUser end";
}
