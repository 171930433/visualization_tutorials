#pragma once
#include "plot/qcustomplot.h"
#include "time_sync.h"
#include <eigen3/Eigen/Dense>
#include <map>
#include <unordered_map>
#include "properties/sub_plot_property.h"

class PlotBase : public QCustomPlot, public ITimeSync {
  Q_OBJECT
public:
  enum Type { None = 0, Matrix, Trajectory, Precision };
  Q_ENUM(Type);

protected:
  PlotBase(QWidget *parent = nullptr);
  void FouseRange(QCPRange const &time_range) override;
  void FocusPoint(double const t0) override;
  std::shared_ptr<QCPGraph> CreateDefaultGraph(QCPAxisRect *rect, QString const &channel_name);
  std::shared_ptr<QCPAxisRect> CreateDefaultRect();
  void setupMatrixDemo(int row, int col);
  void RowColChanged(int const new_row, int const new_col);

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
  QMap<QString, QList<QCPGraph *>> channel_graph_; // 通道名到grap的映射
  QMap<QString, QList<QCPCurve *>> channel_curve_; // 通道名到grap的映射
  // 额外的rect索引,因为this->plotLayout()->rowCount() col 会包含有nullptr
  rviz::MatrixX<QCPAxisRect> all_rects_;
};