#pragma once
#include "plot/plot_base.h"
#include <map>

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

  void AddSeries(QString const &name, QStringList const &field_names);

protected:
  void setupVector3Demo();
  std::map<std::string, QCustomPlot *> all_plots_;
  std::map<int, QCPAxisRect *> all_rects_;
private slots:
  void contextMenuRequest(QPoint pos);
  void ShowSubplot(int const index);
  void mouseWheel();

protected:
  void addRandomGraph();
  DisplaySyncBase *sync_display_ = nullptr;

protected:
  void FocusPoint(double const t0) override;
  void FouseRange(QCPRange const &time_range) {}
  void FoucuPositionByIndex(QCPGraph *single_graph, int const dataIndex);
};