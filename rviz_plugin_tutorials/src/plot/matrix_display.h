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

namespace rviz
{
  class IntProperty;
  class StringProperty;
  class EnumProperty;
  class EditableEnumProperty;
  class BoolProperty;
  class GroupProperty;
  class ColorProperty;
}

using MatrixXQEEnumProp = Eigen::Matrix<rviz::EditableEnumProperty *, Eigen::Dynamic, Eigen::Dynamic>;

class MatrixWidget;
// class QCPCurve;

class MatrixDisplay : public DisplaySyncBase
{
  Q_OBJECT
public:
  MatrixDisplay();
  ~MatrixDisplay() override;
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
  void UpdateChannelName();
  void UpdateFieldName(int const row, int const col);
  void UpdateRow();
  void UpdateCol();
  void ListCurrentChannel(rviz::EditableEnumProperty *topics);

private:
  std::shared_ptr<rviz::EditableEnumProperty> CreateEditEnumProperty(int const row, int const col);

private:
  MatrixWidget *view_ = nullptr;

  MatrixXQEEnumProp fields_prop_;
  rviz::IntProperty *row_prop_ = nullptr;
  rviz::IntProperty *col_prop_ = nullptr;
  rviz::EditableEnumProperty *data_channel_;

private:
  static int object_count_;
};
