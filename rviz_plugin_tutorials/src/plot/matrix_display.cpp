#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>

#include "plot/matrix_display.h"
#include "plot/matrix_widget.h"
#include "protobuf_helper.h"

int MatrixDisplay::object_count_ = 0;
QString MatrixDisplay::generateName()
{
  return QString("MatrixDisplay-p%1").arg(object_count_);
}

MatrixDisplay::MatrixDisplay()
{
  InitPersons();
  setClassId("rviz_plugin_tutorials/MatrixDisplay"); // 和描述文件得一致
  view_ = new MatrixWidget();
  view_->setDisplaySync(this);

  //
  data_channel_ = new rviz::EditableEnumProperty("main_channel", "", "data_channel", this);
  connect(data_channel_, &rviz::EditableEnumProperty::changed, [this]()
          { this->UpdateChannelName(); });
  connect(data_channel_, &rviz::EditableEnumProperty::requestOptions, this, &MatrixDisplay::ListCurrentChannel);

  // ! 需要互相访问,限制范围的set需要在row col都构造完再设置
  row_prop_ = new rviz::IntProperty("row count", 0, "row of mat graphs", this, SLOT(UpdateRow()));
  col_prop_ = new rviz::IntProperty("col count", 0, "col of mat graphs", this, SLOT(UpdateCol()));
  row_prop_->setMin(1);
  row_prop_->setMax(10);
  col_prop_->setMin(1);
  col_prop_->setMax(10);

  ++object_count_;
}

void MatrixDisplay::UpdateChannelName()
{
  auto channel_name = data_channels_->getStdString();
  auto type_name = g_cacher_->GetTypeNameWithChannelName(channel_name);
  auto msg = CreateMessageByName(type_name);
  auto field_names = GetFildNames(*msg);

  for (int i = 0; i < fields_prop_.size(); ++i)
  {
    fields_prop_.data()[i]->clearOptions();
    fields_prop_.data()[i]->addOption("None");

    for (auto const &name : field_names)
    {

      fields_prop_.data()[i]->addOption(name);
    }
  }
}

std::shared_ptr<rviz::EditableEnumProperty> MatrixDisplay::CreateEditEnumProperty(int const row, int const col)
{
  auto when_delete = [this](rviz::EditableEnumProperty *elem)
  { this->takeChild(elem);  delete elem; };
  rviz::EditableEnumProperty *new_field = new rviz::EditableEnumProperty(QString("field-%1-%2").arg(row).arg(col), "", "matrix plot field", this);
  std::shared_ptr<rviz::EditableEnumProperty> result(new_field, when_delete);

  connect(new_field.get(), &Property::changed, [this, row, col]()
          { this->UpdateFieldName(row, col); });

  return result;
}

void MatrixDisplay::ListCurrentChannel(rviz::EditableEnumProperty *topics)
{
  auto const names = g_cacher_->GetChannelNames();
  topics->clearOptions();
  for (auto const &name : names)
  {
    topics->addOptionStd(name);
    // qDebug() << "name = " << QString::fromStdString(name);
  }

  qDebug() << QString("MatrixDisplay::ListCurrentChannel() called ,size= %1 ").arg(names.size());
}

MatrixDisplay::~MatrixDisplay()
{
  if (initialized())
  {
    delete view_;
  }
  --object_count_;
}

void MatrixDisplay::UpdateRow()
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

        fields_prop_(i, j) = CreateEditEnumProperty(i, j);
      }
    }
  }
  else // 删除
  {
    for (int i = new_row; i < old_row; ++i)
    {
      for (int j = 0; j < col; ++j)
      {
        // this->takeChild(fields_prop_(i, j));
        // delete fields_prop_(i, j);
      }
    }
    fields_prop_.conservativeResize(new_row, col); // 删除操作
  }
  qDebug() << QString("col_prop_->getInt()=%1*%2").arg(new_row).arg(col_prop_->getInt());
  view_->UpdatePlotLayout(new_row, col);
}
void MatrixDisplay::UpdateCol()
{

  int const old_col = fields_prop_.cols();
  int const new_col = col_prop_->getInt();
  int const row = row_prop_->getInt();
  if (row == 0)
  {
    return;
  }
  // 增加
  if (old_col < new_col)
  {
    fields_prop_.conservativeResize(row, new_col);
    for (int i = 0; i < row; ++i)
    {
      for (int j = old_col; j < new_col; ++j)
      {
        fields_prop_(i, j) = CreateEditEnumProperty(i, j);
      }
    }
  }
  else // 删除
  {
    for (int i = 0; i < row; ++i)
    {
      for (int j = new_col; j < old_col; ++j)
      {
        // this->takeChild(fields_prop_(i, j));
        // delete fields_prop_(i, j);
      }
    }
    fields_prop_.conservativeResize(row, new_col); // 删除操作
  }
  view_->UpdatePlotLayout(row, new_col);
}

void MatrixDisplay::UpdateFieldName(int const row, int const col)
{
  QString const &field_name = fields_prop_(row, col)->getString();
  if (field_name == "")
  {
    return;
  }
  view_->UpdateFieldName(row, col, field_name);
}

void MatrixDisplay::CreateMatrixPlot(QString const &name, MatrixXQString const &field_names)
{
  view_->CreatePlot(name, field_names);
  // 创建property
  row_prop_->setInt(field_names.rows());
  col_prop_->setInt(field_names.cols());

  for (int i = 0; i < field_names.rows(); ++i)
  {
    for (int j = 0; j < field_names.cols(); ++j)
    {
      fields_prop_(i, j)->setString(field_names(i, j));
    }
  }
}

// Overrides from Display
void MatrixDisplay::onInitialize()
{
  setAssociatedWidget(view_);
}

void MatrixDisplay::update(float dt, float ros_dt)
{
}

void MatrixDisplay::load(const rviz::Config &config)
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
void MatrixDisplay::save(rviz::Config config) const
{
  rviz::Display::save(config);
}

// void MatrixDisplay::AddSeries(QString const &name, QStringList const &field_names)
// {
//   view_->AddSeries(name, field_names);
// }

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(MatrixDisplay, rviz::Display)