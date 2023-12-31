#include <QWidget>

#include "plot/plot_base.h"

PlotBase::PlotBase(QWidget *parent) : QCustomPlot(parent) {
  //
  this->setVisible(false);
}