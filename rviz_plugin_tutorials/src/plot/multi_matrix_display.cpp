#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>

#include "plot/matrix_widget.h"
#include "plot/multi_matrix_display.h"
#include "properties/cached_channel_property.h"
#include "protobuf_helper.h"

RectProperty::RectProperty(MatrixWidget *plot, MultiMatrixDisplay *parent)
    : rviz::BoolProperty("rect", true, "sub plot", parent), plot_(plot) {
  connect(&dataTimer_, SIGNAL(timeout()), this, SLOT(SyncInfo()));
}

void RectProperty::SyncInfo() {
  // 去buffer里面查询通道更新
  // std::string channel_name = data_channel_->getStdString();
  // if (!view_ || channel_name == "") { return; }
  // // 当前时间
  // double t0 = view_->getLastDataTime(channel_name);
  // auto msgs = g_cacher_->GetProtoWithChannleName(channel_name, t0);
  // if (!msgs.empty()) {
  //   view_->AddNewData(channel_name, msgs);
  //   view_->rescaleAxes(); //! 此处目前必
  //   view_->replot();
  //   // qDebug() << QString("add size =  %1").arg(msgs.size());
  // }

  // 去buffer里面查询数据更新
}

void RectProperty::UpdateChannelCount() {
  int const new_count = channels_->size();
  int const old_count = graphs_.size();

  auto when_delete = [this](rviz::SubGraphProperty *elem) {
    this->takeChild(elem);
    delete elem;
  };

  graphs_.conservativeResize(new_count, 1);
  for (int i = old_count; i < new_count; ++i) {
    QString channel_header = (graphs_.size() == 0 ? QString("main_filed") : QString("field-%1").arg(graphs_.size()));
    auto field = new rviz::SubGraphProperty(channel_header, "", "field_name", this);
    // field->setChannelProperty((*channels_)(i, 0).get());
    field->setChannelProperty(channels_->operator()(i, 0).get());
    std::shared_ptr<rviz::SubGraphProperty> result(field, when_delete);
    connect(field, &rviz::Property::changed, [this, result]() { this->UpdateFieldName(result); });
    graphs_(i, 0) = result;
  }
}

void RectProperty::UpdateFieldName(std::shared_ptr<rviz::SubGraphProperty> sub_graph) {
  QString const &field_name = sub_graph->getString();
  qDebug() << QString("UpdateFieldName = %1").arg(field_name);

  dataTimer_.stop();
  if (!field_name.isEmpty()) {
    sub_graph->graph() = plot_->CreateGraphByFieldName(row_, col_, field_name);
  } else {
    sub_graph->graph().reset();
  }
  plot_->replot();

  dataTimer_.start(500);
}

int MultiMatrixDisplay::object_count_ = 0;
QString MultiMatrixDisplay::generateName() { return QString("MultiMatrixDisplay-p%1").arg(object_count_); }

MultiMatrixDisplay::MultiMatrixDisplay() {
  InitPersons();
  setClassId("rviz_plugin_tutorials/MultiMatrixDisplay"); // 和描述文件得一致
  view_ = new MatrixWidget();
  view_->setDisplaySync(this);

  //
  counts_prop_ =
      new rviz::IntProperty("channel counts", 0, "the number of channel counts", this, SLOT(UpdateChannelCount()));

  // ! 需要互相访问,限制范围的set需要在row col都构造完再设置
  row_prop_ = new rviz::IntProperty("row count", 0, "row of mat graphs", this, SLOT(UpdateRow()));
  col_prop_ = new rviz::IntProperty("col count", 0, "col of mat graphs", this, SLOT(UpdateCol()));

  // 会自动生成rect-0-0
  row_prop_->setMin(1);
  row_prop_->setMax(10);
  col_prop_->setMin(1);
  col_prop_->setMax(10);

  //
  counts_prop_->setMin(1);
  counts_prop_->setMax(10);

  ++object_count_;
}

MultiMatrixDisplay::~MultiMatrixDisplay() {
  if (initialized()) { delete view_; }
  --object_count_;
}

void MultiMatrixDisplay::UpdateChannelCount() {
  int const new_count = counts_prop_->getInt();
  int const old_count = data_channels_.size();

  data_channels_.conservativeResize(new_count, 1);
  auto when_delete = [this](rviz::CachedChannelProperty *elem) {
    this->takeChild(elem);
    delete elem;
  };

  // 新增加元素
  for (int i = old_count; i < new_count; ++i) {
    QString channel_header =
        (data_channels_.size() == 0 ? QString("main_channel") : QString("data_channel-%1").arg(data_channels_.size()));

    auto single_changel = new rviz::CachedChannelProperty(channel_header, "", "data_channel", counts_prop_);
    std::shared_ptr<rviz::CachedChannelProperty> new_single_changel(single_changel, when_delete);
    data_channels_(i, 0) = new_single_changel;
  }
}

std::shared_ptr<RectProperty> MultiMatrixDisplay::CreateRectProperty(int const row, int const col) {
  auto when_delete = [this](RectProperty *elem) {
    this->takeChild(elem);
    delete elem;
  };

  std::shared_ptr<RectProperty> rect_prop(new RectProperty(view_, this), when_delete);
  rect_prop->setName(QString("rect-%1-%2").arg(row).arg(col));
  rect_prop->UpdateChannelCount();
  rect_prop->setLayout(row, col);
  auto when_channel_count_changed = [this, rect_prop]() { rect_prop->UpdateChannelCount(); };

  connect(counts_prop_, &rviz::IntProperty::changed, when_channel_count_changed);
  return rect_prop;
}

void MultiMatrixDisplay::UpdateRow() {
  int const old_row = fields_prop_.rows();
  int const new_row = row_prop_->getInt();
  int const col = col_prop_->getInt();
  if (col == 0) { return; }

  fields_prop_.conservativeResize(new_row, col);
  // 增加
  for (int i = old_row; i < new_row; ++i) {
    for (int j = 0; j < col; ++j) {
      fields_prop_(i, j) = CreateRectProperty(i, j);
    }
  }

  qDebug() << QString("col_prop_->getInt()=%1*%2").arg(new_row).arg(col_prop_->getInt());
  view_->UpdatePlotLayout(new_row, col);
}
void MultiMatrixDisplay::UpdateCol() {
  int const old_col = fields_prop_.cols();
  int const new_col = col_prop_->getInt();
  int const row = row_prop_->getInt();
  if (row == 0) { return; }
  fields_prop_.conservativeResize(row, new_col);
  // 增加
  for (int i = 0; i < row; ++i) {
    for (int j = old_col; j < new_col; ++j) {
      fields_prop_(i, j) = CreateRectProperty(i, j);
    }
  }

  view_->UpdatePlotLayout(row, new_col);
}

void MultiMatrixDisplay::CreateMatrixPlot(QString const &name, MatrixXQString const &field_names) {
  view_->CreatePlot(name, field_names);
  // 创建property
  row_prop_->setInt(field_names.rows());
  col_prop_->setInt(field_names.cols());

  for (int i = 0; i < field_names.rows(); ++i) {
    for (int j = 0; j < field_names.cols(); ++j) {
      // fields_prop_(i, j)->setString(field_names(i, j));
    }
  }
}

// Overrides from Display
void MultiMatrixDisplay::onInitialize() { setAssociatedWidget(view_); }

void MultiMatrixDisplay::update(float dt, float ros_dt) {}

void MultiMatrixDisplay::load(const rviz::Config &config) {
  int row = 0;
  if (config.mapGetInt("row count", &row)) { row_prop_->setInt(row); }
  int col = 0;
  if (config.mapGetInt("col count", &col)) { col_prop_->setInt(col); }
  rviz::Display::load(config);
}
void MultiMatrixDisplay::save(rviz::Config config) const { rviz::Display::save(config); }

// void MultiMatrixDisplay::AddSeries(QString const &name, QStringList const &field_names)
// {
//   view_->AddSeries(name, field_names);
// }

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(MultiMatrixDisplay, rviz::Display)