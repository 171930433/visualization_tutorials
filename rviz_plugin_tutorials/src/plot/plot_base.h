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
    Matrix,
    Trajectory,
    Precision
  };
  Q_ENUM(Type);

protected:
  PlotBase(QWidget *parent = nullptr);
  void FouseRange(QCPRange const &time_range) override;
  void FocusPoint(double const t0) override;
  QCPGraph *CreateDefaultGraph(QCPAxisRect *rect);
  QCPAxisRect *CreateDefaultRect();
  void setupMatrixDemo(int row, int col);

protected:
  void keyPressEvent(QKeyEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;

public:
protected slots:
  void onSelectionChangedByUser(); // 相应用户的选择改变

protected:
  Type type_ = Type::None;
  QSharedPointer<QCPAxisTickerDateTime> dateTicker_;
};