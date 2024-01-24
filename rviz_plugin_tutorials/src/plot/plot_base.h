#pragma once
#include "plot/qcustomplot.h"
#include "time_sync.h"
#include <eigen3/Eigen/Dense>
#include <map>

namespace rviz {
class CachedChannelProperty;
class FieldListProperty;
} // namespace rviz

class SubGraphPlot {
public:
  SubGraphPlot();
  ~SubGraphPlot();
  QString getString();
  void setString(QString const &str);

public:
  std::shared_ptr<rviz::FieldListProperty> field_prop_ = nullptr;
  std::shared_ptr<QCPGraph> graph_ = nullptr;
};

class PlotBase : public QCustomPlot, public ITimeSync {
  Q_OBJECT
public:
  enum Type { None = 0, Matrix, Trajectory, Precision };
  Q_ENUM(Type);

protected:
  PlotBase(QWidget *parent = nullptr);
  void FouseRange(QCPRange const &time_range) override;
  void FocusPoint(double const t0) override;
  std::shared_ptr<QCPGraph> CreateDefaultGraph(QCPAxisRect *rect);
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