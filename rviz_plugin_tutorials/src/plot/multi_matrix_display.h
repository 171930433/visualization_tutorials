#pragma once

#include "properties/cached_channel_property.h"
#include "properties/sub_plot_property.h"
#include <rviz/display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>

#include "display_sync_base.h"
#include "trajectory_widget.h"
#include <deque>
#include <eigen3/Eigen/Dense>

namespace rviz {
class IntProperty;
class StringProperty;
class GroupProperty;
class ColorProperty;
} // namespace rviz
class MatrixWidget;
class MatrixXChannel;
class MultiMatrixDisplay;

using MatrixXQString = Eigen::Matrix<QString, Eigen::Dynamic, Eigen::Dynamic>;

class RectProperty : public rviz::BoolProperty {
  Q_OBJECT

public:
  RectProperty(MatrixWidget *plot, MultiMatrixDisplay *parent);
  void setLayout(int const row, int const col) { row_ = row, col_ = col; }
public Q_SLOTS:
  void UpdateChannelCount();

protected:
  rviz::MatrixXSubGraph graphs_;
  MultiMatrixDisplay *parent_;
  MatrixWidget *plot_;
  int row_, col_;
};
// using MatrixXRectProp = Eigen::Matrix<std::shared_ptr<RectProperty>, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixXRectProp = rviz::MatrixX<RectProperty>;

// class QCPCurve;

class MultiMatrixDisplay : public DisplaySyncBase {
  Q_OBJECT
public:
  MultiMatrixDisplay();
  ~MultiMatrixDisplay() override;
  static QString generateName(); // 防止用户重建时名称重复

public:
  // void AddSeries(QString const &name, QStringList const &field_names);
  void CreateMatrixPlot(QString const &name, MatrixXQString const &field_names);

  // Overrides from Display
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  void onInitialize() override;
  void update(float dt, float ros_dt) override;

private Q_SLOTS:
  void UpdateRow();
  void UpdateCol();
  void UpdateChannelCount();
  void SyncInfo();
public Q_SLOTS:

  void UpdateFieldName(rviz::SubGraphProperty *sub_graph, int const row, int const col);

private:
  std::shared_ptr<RectProperty> CreateRectProperty(int const row, int const col);

private:
  MatrixWidget *view_ = nullptr;
  QTimer dataTimer_; // 检查是否有数据更新

  rviz::IntProperty *row_prop_ = nullptr;
  rviz::IntProperty *col_prop_ = nullptr;
  rviz::IntProperty *counts_prop_ = nullptr; // 通道数目

private:
  static int object_count_;

  rviz::MatrixXChannel data_channels_;
  MatrixXRectProp fields_prop_;
  friend RectProperty;
};
