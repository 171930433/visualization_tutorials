#pragma once
#include <map>
#include <eigen3/Eigen/Dense>
#include "plot/qcustomplot.h"
#include "time_sync.h"

using MatrixXQString = Eigen::Matrix<QString, Eigen::Dynamic, Eigen::Dynamic>;
// Eigen::Matrix<QCPAxisRect *, Eigen::Dynamic, Eigen::Dynamic> rects_;
// Eigen::Matrix<QCPGraph *, Eigen::Dynamic, Eigen::Dynamic> graphs_;

class PlotBase : public QCustomPlot, public ITimeSync
{
  Q_OBJECT
public:
  enum Type
  {
    None = 0,
    Matrix = 1,
    Trajectory = 2
  };
  Q_ENUM(Type);

protected:
  PlotBase(QWidget *parent = nullptr);
  void keyPressEvent(QKeyEvent *event) override;

public:
  virtual void SyncDataAndView() = 0;
protected slots:
  void onBeforeReplot();

protected:
  Type type_ = Type::None;
  QCPRange time_range_; // 当前所有数据的范围
};