#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>

#include "plot/multi_matrix_display.h"
#include "plot/matrix_widget.h"
#include "protobuf_helper.h"

RectProperty::RectProperty(MatrixWidget *plot, Property *parent)
    : rviz::BoolProperty("rect", true, "sub plot", parent),
      plot_(plot)
{
}

void RectProperty::UpdateFieldNames(int const count, QStringList const &names)
{
  graphs_prop_[count]->clearOptions();
  graphs_prop_[count]->addOption("None");

  for (auto const &name : names)
  {
    graphs_prop_[count]->addOption(name);
  }
}

void RectProperty::UpdateChannelCount(int const count)
{
  int const new_count = count;
  int const old_count = graphs_prop_.size();

  if (new_count < old_count) // 删除元素
  {
    for (int i = new_count; i < old_count; ++i)
    {
      this->takeChild(graphs_prop_.back().get());
      graphs_prop_.pop_back();
    }
  }
  else
  {
    for (int i = old_count; i < new_count; ++i)
    {
      QString channel_header = (graphs_prop_.size() == 0 ? QString("main_filed") : QString("field-%1").arg(graphs_prop_.size()));
      auto field = std::make_shared<rviz::EditableEnumProperty>(channel_header, "", "field_name", this, SLOT(UpdateChannel()));
      // connect(field.get(), &rviz::EditableEnumProperty::requestOptions, this, &MultiMatrixDisplay::ListCurrentChannel);
      graphs_prop_.push_back(field);
    }
  }
}

int MultiMatrixDisplay::object_count_ = 0;
QString MultiMatrixDisplay::generateName()
{
  return QString("MultiMatrixDisplay-p%1").arg(object_count_);
}

MultiMatrixDisplay::MultiMatrixDisplay()
{
  InitPersons();
  setClassId("rviz_plugin_tutorials/MultiMatrixDisplay"); // 和描述文件得一致
  view_ = new MatrixWidget();
  view_->setDisplaySync(this);

  //
  counts_prop_ = new rviz::IntProperty("channel counts", 0, "the number of channel counts", this, SLOT(UpdateChannelCount()));

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

MultiMatrixDisplay::~MultiMatrixDisplay()
{
  if (initialized())
  {
    delete view_;
  }
  --object_count_;
}

void MultiMatrixDisplay::UpdateChannelName(int const row)
{
  auto channel_name = data_channels_[row]->getStdString();
  auto type_name = g_cacher_->GetTypeNameWithChannelName(channel_name);
  auto msg = CreateMessageByName(type_name);
  auto field_names = GetFildNames(*msg);

  for (int i = 0; i < fields_prop_.size(); ++i)
  {
    fields_prop_.data()[i]->UpdateFieldNames(row, field_names);
  }
}

void MultiMatrixDisplay::UpdateChannelCount()
{

  int const new_count = counts_prop_->getInt();
  int const old_count = data_channels_.size();

  if (new_count < old_count) // 删除元素
  {
    for (int i = new_count; i < old_count; ++i)
    {
      this->takeChild(data_channels_.back().get());
      data_channels_.pop_back();
    }
  }
  else
  {
    for (int i = old_count; i < new_count; ++i)
    {
      QString channel_header = (data_channels_.size() == 0 ? QString("main_channel") : QString("data_channel-%1").arg(data_channels_.size()));
      auto single_changel = std::make_shared<rviz::EditableEnumProperty>(channel_header, "", "data_channel", counts_prop_);
      connect(single_changel.get(), &rviz::EditableEnumProperty::changed, [this, i]()
              { this->UpdateChannelName(i); });
      connect(single_changel.get(), &rviz::EditableEnumProperty::requestOptions, this, &MultiMatrixDisplay::ListCurrentChannel);
      data_channels_.push_back(single_changel);
    }
  }
}

void MultiMatrixDisplay::ListCurrentChannel(rviz::EditableEnumProperty *topics)
{
  auto const names = g_cacher_->GetChannelNames();
  topics->clearOptions();
  for (auto const &name : names)
  {
    topics->addOptionStd(name);
    // qDebug() << "name = " << QString::fromStdString(name);
  }

  qDebug() << QString("MultiMatrixDisplay::ListCurrentChannel() called ,size= %1 ").arg(names.size());
}

std::shared_ptr<RectProperty> MultiMatrixDisplay::CreateRectProperty(int const row, int const col)
{
  auto when_delete = [this](RectProperty *elem)
  { this->takeChild(elem);  delete elem; };

  std::shared_ptr<RectProperty> rect_prop(new RectProperty(view_, this), when_delete);
  rect_prop->setName(QString("rect-%1-%2").arg(row).arg(col));
  rect_prop->UpdateChannelCount(counts_prop_->getInt());

  auto when_channel_count_changed = [this, rect_prop]()
  { rect_prop->UpdateChannelCount(this->counts_prop_->getInt()); };

  connect(counts_prop_, &rviz::IntProperty::changed, when_channel_count_changed);
  return rect_prop;
}

void MultiMatrixDisplay::UpdateRow()
{
  int const old_row = fields_prop_.rows();
  int const new_row = row_prop_->getInt();
  int const col = col_prop_->getInt();
  if (col == 0)
  {
    return;
  }

  fields_prop_.conservativeResize(new_row, col);
  // 增加
  if (old_row < new_row)
  {
    for (int i = old_row; i < new_row; ++i)
    {
      for (int j = 0; j < col; ++j)
      {
        fields_prop_(i, j) = CreateRectProperty(i, j);
      }
    }
  }

  qDebug() << QString("col_prop_->getInt()=%1*%2").arg(new_row).arg(col_prop_->getInt());
  view_->UpdatePlotLayout(new_row, col);
}
void MultiMatrixDisplay::UpdateCol()
{

  int const old_col = fields_prop_.cols();
  int const new_col = col_prop_->getInt();
  int const row = row_prop_->getInt();
  if (row == 0)
  {
    return;
  }
  fields_prop_.conservativeResize(row, new_col);
  // 增加
  if (old_col < new_col)
  {
    for (int i = 0; i < row; ++i)
    {
      for (int j = old_col; j < new_col; ++j)
      {
        fields_prop_(i, j) = CreateRectProperty(i, j);
      }
    }
  }

  view_->UpdatePlotLayout(row, new_col);
}

void MultiMatrixDisplay::UpdateFieldName(int const row, int const col)
{
  // QString const &field_name = fields_prop_(row, col)->getString();
  // if (field_name == "")
  // {
  //   return;
  // }
  // view_->UpdateFieldName(row, col, field_name);
}

void MultiMatrixDisplay::CreateMatrixPlot(QString const &name, MatrixXQString const &field_names)
{
  view_->CreatePlot(name, field_names);
  // 创建property
  row_prop_->setInt(field_names.rows());
  col_prop_->setInt(field_names.cols());

  for (int i = 0; i < field_names.rows(); ++i)
  {
    for (int j = 0; j < field_names.cols(); ++j)
    {
      // fields_prop_(i, j)->setString(field_names(i, j));
    }
  }
}

// Overrides from Display
void MultiMatrixDisplay::onInitialize()
{
  setAssociatedWidget(view_);
}

void MultiMatrixDisplay::update(float dt, float ros_dt)
{
}

void MultiMatrixDisplay::load(const rviz::Config &config)
{
  int row = 0;
  if (config.mapGetInt("row count", &row))
  {
    row_prop_->setInt(row);
  }
  int col = 0;
  if (config.mapGetInt("col count", &col))
  {
    col_prop_->setInt(col);
  }
  rviz::Display::load(config);
}
void MultiMatrixDisplay::save(rviz::Config config) const
{
  rviz::Display::save(config);
}

// void MultiMatrixDisplay::AddSeries(QString const &name, QStringList const &field_names)
// {
//   view_->AddSeries(name, field_names);
// }

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(MultiMatrixDisplay, rviz::Display)