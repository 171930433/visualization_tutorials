#pragma once

#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/display.h>

#include "display_sync_base.h"
#include <deque>
#include <eigen3/Eigen/Dense>
#include "properties/sub_plot_property.h"


namespace rviz {
class IntProperty;
class StringProperty;
class EnumProperty;
class EditableEnumProperty;
class CachedChannelProperty;
class FieldListProperty;
class BoolProperty;
class GroupProperty;
class ColorProperty;
} // namespace rviz

class MatrixWidget;
using MatrixXQString = Eigen::Matrix<QString, Eigen::Dynamic, Eigen::Dynamic>;


class MatrixDisplay : public DisplaySyncBase {
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
  void SyncInfo();

private:
  std::shared_ptr<rviz::SubGraphProperty> CreateSubGraphPlot(int const row, int const col);

private:
  MatrixWidget *view_ = nullptr;

  rviz::MatrixXSubGraph fields_prop_;
  rviz::IntProperty *row_prop_ = nullptr;
  rviz::IntProperty *col_prop_ = nullptr;
  rviz::CachedChannelProperty *data_channel_;
  QTimer dataTimer_; // 检查是否有数据更新

private:
  static int object_count_;
};
