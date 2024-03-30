#include "plot/matrix_display.h"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/visualization_manager.hpp>

#include "plot/matrix_widget.h"
#include "properties/cached_channel_property.h"
#include "protobuf_helper.h"

int MatrixDisplay::object_count_ = 0;
QString MatrixDisplay::generateName() { return QString("MatrixDisplay-p%1").arg(object_count_); }

MatrixDisplay::MatrixDisplay() {
  using namespace rviz_common::properties;
  InitPersons();
  setClassId("rviz_plugin_tutorials/MatrixDisplay"); // 和描述文件得一致
  view_ = new MatrixWidget();
  view_->setDisplaySync(this);

  //
  data_channel_ = new CachedChannelProperty("channel_name", "", "data_channel", this);
  connect(data_channel_, &CachedChannelProperty::changed, [this]() { this->UpdateChannelName(); });

  // ! 需要互相访问,限制范围的set需要在row col都构造完再设置
  row_prop_ = new IntProperty("row count", 0, "row of mat graphs", this, SLOT(UpdateRow()));
  col_prop_ = new IntProperty("col count", 0, "col of mat graphs", this, SLOT(UpdateCol()));
  row_prop_->setMin(1);
  row_prop_->setMax(10);
  col_prop_->setMin(1);
  col_prop_->setMax(10);

  ++object_count_;
  connect(&dataTimer_, SIGNAL(timeout()), this, SLOT(SyncInfo()));
}

void MatrixDisplay::SyncInfo() {
  // 去buffer里面查询通道更新
  std::string channel_name = data_channel_->getStdString();
  if (!view_ || channel_name == "") { return; }
  // 当前时间
  double t0 = view_->getLastDataTime(channel_name);
  auto msgs = g_cacher_->GetProtoWithChannleName(channel_name, t0);
  if (!msgs.empty()) {
    view_->AddNewData(channel_name, msgs);
    view_->rescaleAxes(); //! 此处目前必
    view_->replot();
    // qDebug() << QString("add size =  %1").arg(msgs.size());
  }

  // 去buffer里面查询数据更新
}

void MatrixDisplay::UpdateChannelName() {
  dataTimer_.stop();

  // 需要重置所有的graph

  dataTimer_.start(500);
}

std::shared_ptr<rviz_common::properties::SubGraphProperty> MatrixDisplay::CreateSubGraphPlot(int const row, int const col) {
  auto when_delete = [this](rviz_common::properties::SubGraphProperty *elem) {
    this->takeChild(elem);
    delete elem;
  };
  //
  auto *new_field = new rviz_common::properties::SubGraphProperty(QString("field-%1-%2").arg(row).arg(col), "", "matrix plot field", this);
  new_field->setChannelProperty(data_channel_);
  std::shared_ptr<rviz_common::properties::SubGraphProperty> result(new_field, when_delete);
  connect(new_field, &Property::changed, [this, row, col]() { this->UpdateFieldName(row, col); });
  //
  qDebug() << QString("created row=%1,col=%2").arg(row).arg(col);
  return result;
}

MatrixDisplay::~MatrixDisplay() {
  if (initialized()) { delete view_; }
  --object_count_;
}

void MatrixDisplay::UpdateRow() {
  int const old_row = fields_prop_.rows();
  int const new_row = row_prop_->getInt();
  int const col = col_prop_->getInt();
  if (col == 0) { return; }

  fields_prop_.resize(new_row, col);

  // 增加
  for (int i = old_row; i < new_row; ++i) {
    for (int j = 0; j < col; ++j) {
      fields_prop_(i, j) = CreateSubGraphPlot(i, j);
    }
  }

  qDebug() << QString("col_prop_->getInt()=%1*%2").arg(new_row).arg(col);
  view_->UpdatePlotLayout(new_row, col);
}
void MatrixDisplay::UpdateCol() {
  int const old_col = fields_prop_.cols();
  int const new_col = col_prop_->getInt();
  int const row = row_prop_->getInt();
  if (row == 0) { return; }
  fields_prop_.resize(row, new_col);
  // 增加
  for (int i = 0; i < row; ++i) {
    for (int j = old_col; j < new_col; ++j) {
      fields_prop_(i, j) = CreateSubGraphPlot(i, j);
    }
  }

  view_->UpdatePlotLayout(row, new_col);
}

void MatrixDisplay::UpdateFieldName(int const row, int const col) {
  QString const &field_name = fields_prop_(row, col)->getString();
  QString const &channel_name = fields_prop_(row, col)->getChannelName();
  qDebug() << QString("UpdateFieldName = %1").arg(field_name);

  if (!fields_prop_(row, col)) { return; }

  dataTimer_.stop();
  if (!field_name.isEmpty()) {
    fields_prop_(row, col)->graph() = view_->CreateGraphByFieldName(row, col, channel_name, field_name);
  } else {
    fields_prop_(row, col)->graph().reset();
  }
  view_->replot();

  dataTimer_.start(500);
}

void MatrixDisplay::CreateMatrixPlot(QString const &name, MatrixXQString const &field_names) {
  view_->CreatePlot(name, field_names);
  // 创建property
  data_channel_->setString(name);
  row_prop_->setInt(field_names.rows());
  col_prop_->setInt(field_names.cols());

  for (int i = 0; i < field_names.rows(); ++i) {
    for (int j = 0; j < field_names.cols(); ++j) {
      fields_prop_(i, j)->setString(field_names(i, j));
    }
  }
}

// Overrides from Display
void MatrixDisplay::onInitialize() { setAssociatedWidget(view_); }

void MatrixDisplay::update(float dt, float ros_dt) {}

void MatrixDisplay::load(const rviz_common::Config &config) {
  QString channel_name;
  if (config.mapGetString("main_channel", &channel_name)) { data_channel_->setString(channel_name); }
  int row = 0;
  if (config.mapGetInt("row count", &row)) { row_prop_->setInt(row); }
  int col = 0;
  if (config.mapGetInt("col count", &col)) { col_prop_->setInt(col); }
  rviz_common::Display::load(config);
}
void MatrixDisplay::save(rviz_common::Config config) const { rviz_common::Display::save(config); }

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MatrixDisplay, rviz_common::Display)