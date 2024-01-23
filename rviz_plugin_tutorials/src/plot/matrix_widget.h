#pragma once
#include "plot/plot_base.h"
#include <map>
#include <deque>
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
  void CreateGraphByFieldName(int const row, int const col, QString const &field_name);

  std::deque<std::shared_ptr<QCPGraph>> AddGraphInRect(int const row, int const col, int count);

protected:
protected:
  std::map<int, QCPAxisRect *> all_rects_;

private slots:
  void contextMenuRequest(QPoint pos);
  void ShowSubplot(int const index);
  void mouseWheel();

protected:
  void RowChanged(int const new_row);
  void ColChanged(int const new_col);

protected:
};