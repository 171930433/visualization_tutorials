#pragma once
#include <map>
#include "plot/qcustomplot.h"

class PlotBase : public QCustomPlot {
  Q_OBJECT
 public:
  enum Type { None = 0, Matrix = 1, Trajectory = 2 };
  Q_ENUM(Type);

 protected:
  PlotBase(QWidget *parent = nullptr);

 public:
  virtual void SyncDataAndView() = 0;

 protected:
  Type type_ = Type::None;
};