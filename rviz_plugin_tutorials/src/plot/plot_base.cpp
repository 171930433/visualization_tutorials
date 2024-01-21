#include <QWidget>

#include "plot/plot_base.h"

PlotBase::PlotBase(QWidget *parent) : QCustomPlot(parent)
{
  //
  this->setVisible(false);
  connect(this, &QCustomPlot::beforeReplot, this, &PlotBase::onBeforeReplot);
}

void PlotBase::onBeforeReplot()
{
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
