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

  void CreatePlot(QString const &name, MatrixXQString const &field_names);
  void UpdatePlotLayout(int const row, int const col);
  void UpdateFieldName(int const row, int const col, QString const &field_name);

protected:

protected:
  void setupMatrixDemo(int row, int col);
  std::map<int, QCPAxisRect *> all_rects_;
  
private slots:
  void contextMenuRequest(QPoint pos);
  void ShowSubplot(int const index);
  void mouseWheel();

protected:
  QCPGraph *CreateDefaultGraph(QCPAxisRect *rect);
  QCPAxisRect *CreateDefaultRect();
  void RowChanged(int const new_row);
  void ColChanged(int const new_col);
  DisplaySyncBase *sync_display_ = nullptr;
  QSharedPointer<QCPAxisTickerDateTime> dateTicker_;

protected:
};