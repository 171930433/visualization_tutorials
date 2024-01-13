#pragma once
#include "plot/plot_base.h"
#include <map>
#include <eigen3/Eigen/Dense>

class QWidget;
class DisplaySyncBase;
class MatrixWidget : public PlotBase
{
  Q_OBJECT

public:
  MatrixWidget(QWidget *parent = nullptr);
  void SyncDataAndView() override {}
  void setDisplaySync(DisplaySyncBase *sync_display) { sync_display_ = sync_display; }
  DisplaySyncBase *getDisplaySync() override;

  void CreatePlot(QString const &name, MatrixXQString const &field_names);
  void UpdatePlotLayout(int const row, int const col);
  void UpdateFieldName(int const row, int const col, QString const &field_name);

protected:
  void keyPressEvent(QKeyEvent *event) override;

protected:
  void setupVector3Demo();
  void setupMatrixDemo(int row, int col);
  std::map<std::string, QCustomPlot *> all_plots_;
  std::map<int, QCPAxisRect *> all_rects_;
  Eigen::Matrix<QCPAxisRect *, Eigen::Dynamic, Eigen::Dynamic> rects_;
  Eigen::Matrix<QCPGraph *, Eigen::Dynamic, Eigen::Dynamic> graphs_;
private slots:
  void contextMenuRequest(QPoint pos);
  void ShowSubplot(int const index);
  void mouseWheel();

protected:
  void addRandomGraph();
  QCPGraph *CreateDefaultGraph(QCPAxisRect *rect);
  QCPAxisRect *CreateDefaultRect();
  DisplaySyncBase *sync_display_ = nullptr;
  QSharedPointer<QCPAxisTickerDateTime> dateTicker_;

protected:
  void FocusPoint(double const t0) override;
  void FouseRange(QCPRange const &time_range) {}
  void FoucuPositionByIndex(QCPGraph *single_graph, int const dataIndex);
};