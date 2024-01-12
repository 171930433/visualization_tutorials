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
  class BoolProperty;
  class GroupProperty;
  class ColorProperty;
}

using MatrixXQStringProp = Eigen::Matrix<rviz::StringProperty *, Eigen::Dynamic, Eigen::Dynamic>;

class MatrixWidget;
// class QCPCurve;

class MatrixDisplay : public DisplaySyncBase
{
  Q_OBJECT
public:
  MatrixDisplay();
  ~MatrixDisplay() override;

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

private:
  MatrixWidget *view_ = nullptr;

  MatrixXQStringProp fields_prop_;
  rviz::IntProperty* row_prop_ = nullptr;
  rviz::IntProperty* col_prop_ = nullptr;

  rviz::StringProperty *field_prop_[3] = {nullptr, nullptr, nullptr};
};
