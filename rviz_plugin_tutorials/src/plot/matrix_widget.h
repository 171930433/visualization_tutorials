#pragma once
#include "cacher/cacher.h"
#include "plot/plot_base.h"
#include <deque>
#include <eigen3/Eigen/Dense>
#include <map>

class QWidget;
class DisplaySyncBase;
using MatrixXQString = Eigen::Matrix<QString, Eigen::Dynamic, Eigen::Dynamic>;

class MatrixWidget : public PlotBase {
  Q_OBJECT

public:
  MatrixWidget(QWidget *parent = nullptr);

  void CreatePlot(QString const &name, MatrixXQString const &field_names);
  void UpdatePlotLayout(int const row, int const col);
  std::shared_ptr<QCPGraph>
  CreateGraphByFieldName(int const row, int const col, QString const &channel_name, QString const &field_name);

  double getLastDataTime(std::string const &channel_name) const;
  void AddNewData(std::string const &channel_name,
                  std::map<size_t, sp_cPbMsg> const &new_data,
                  int const channel_index = 0); // channel_index 表示第几个通道

protected:
protected:

private slots:
  void contextMenuRequest(QPoint pos);
  void ShowSubplot(int const index);
  void mouseWheel();

protected:
  void RowChanged(int const new_row);
  void ColChanged(int const new_col);

protected:
  CacherBuffer channel_msgs_; // 所有更新的数据缓存
};