#pragma once

// #include <rviz/properties/color_property.h>
// #include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>

// #include <rviz/properties/vector_property.h>
// #include <rviz/properties/enum_property.h>
// #include <rviz/properties/tf_frame_property.h>
#include <rviz/display.h>

#include "display_sync_base.h"
#include "trajectory_widget.h"
#include <deque>
#include <eigen3/Eigen/Dense>

namespace rviz {
class IntProperty;
class StringProperty;
class EditableEnumProperty;
class CachedChannelProperty;
// class BoolProperty;
class GroupProperty;
class ColorProperty;
} // namespace rviz
class MatrixWidget;

using MatrixXQString = Eigen::Matrix<QString, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixXQStringProp = Eigen::Matrix<rviz::StringProperty *, Eigen::Dynamic, Eigen::Dynamic>;

class RectProperty : public rviz::BoolProperty {
  Q_OBJECT

public:
  RectProperty(MatrixWidget *plot, Property *parent = nullptr);
  void setLayout(int const row, int const col) { row_ = row, col_ = col_; }
public Q_SLOTS:

  void UpdateChannelCount(std::deque<std::shared_ptr<rviz::CachedChannelProperty>> const &channels);
  void UpdateFieldNames(int const count, QStringList const &names);

protected:
  std::deque<std::shared_ptr<QCPGraph>> graphs_;
  std::deque<std::shared_ptr<rviz::FieldListProperty>> graphs_prop_;
  MatrixWidget *plot_;
  int row_, col_;
};
using MatrixXRectProp = Eigen::Matrix<std::shared_ptr<RectProperty>, Eigen::Dynamic, Eigen::Dynamic>;

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
  void UpdateFieldName(int const row, int const col);
  void UpdateRow();
  void UpdateCol();
  void UpdateChannelCount();
  void UpdateChannelName(int const row);

private:
  std::shared_ptr<RectProperty> CreateRectProperty(int const row, int const col);

private:
  MatrixWidget *view_ = nullptr;

  rviz::IntProperty *row_prop_ = nullptr;
  rviz::IntProperty *col_prop_ = nullptr;
  rviz::IntProperty *counts_prop_ = nullptr; // 通道数目

private:
  static int object_count_;
  std::deque<std::shared_ptr<rviz::CachedChannelProperty>> data_channels_;
  MatrixXRectProp fields_prop_;
  friend RectProperty;
};
