#pragma once

#include "properties/cached_channel_property.h"
#include "properties/sub_plot_property.h"
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>

#include "display_sync_base.h"
#include "trajectory_widget.h"
#include <deque>
#include <eigen3/Eigen/Dense>

namespace rviz_common {
namespace properties {

class IntProperty;
class StringProperty;
class GroupProperty;
class ColorProperty;

} // namespace properties
} // namespace rviz_common

class MatrixWidget;
class MatrixXChannel;
class MultiMatrixDisplay;

using MatrixXQString = Eigen::Matrix<QString, Eigen::Dynamic, Eigen::Dynamic>;

namespace rviz_common {
namespace properties {
class RectProperty : public BoolProperty {
  Q_OBJECT

public:
  RectProperty(MatrixWidget *plot, MultiMatrixDisplay *parent);
  void setLayout(int const row, int const col) { row_ = row, col_ = col; }
public Q_SLOTS:
  void UpdateChannelCount();

protected:
  VectorXSubGraph graphs_;
  MultiMatrixDisplay *parent_;
  MatrixWidget *plot_;
  int row_, col_;
};

using MatrixXRectProp = MatrixX<RectProperty>;


} // namespace properties
} // namespace rviz_common

// using MatrixXRectProp = Eigen::Matrix<std::shared_ptr<RectProperty>, Eigen::Dynamic, Eigen::Dynamic>;

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
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;
  void onInitialize() override;
  void update(float dt, float ros_dt) override;

private Q_SLOTS:
  void UpdateRowCol();
  void UpdateChannelCount();
  void SyncInfo();
public Q_SLOTS:

  void UpdateFieldName(rviz_common::properties::SubGraphProperty *sub_graph, int const row, int const col);

private:
  std::shared_ptr<rviz_common::properties::RectProperty> CreateRectProperty(int const row, int const col);

private:
  MatrixWidget *view_ = nullptr;
  QTimer dataTimer_; // 检查是否有数据更新

  rviz_common::properties::IntProperty *row_prop_ = nullptr;
  rviz_common::properties::IntProperty *col_prop_ = nullptr;
  rviz_common::properties::IntProperty *counts_prop_ = nullptr; // 通道数目

private:
  static int object_count_;

  rviz_common::properties::VectorXChannel data_channels_;
  rviz_common::properties::MatrixXRectProp fields_prop_;
  friend rviz_common::properties::RectProperty;
};
