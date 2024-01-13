#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>

#include "plot/matrix_display.h"
#include "plot/matrix_widget.h"
#include "protobuf_helper.h"

MatrixDisplay::MatrixDisplay()
{
  InitPersons();
  setClassId("rviz_plugin_tutorials/MatrixDisplay"); // 和描述文件得一致
  view_ = new MatrixWidget();
  view_->setDisplaySync(this);

  row_prop_ = new rviz::IntProperty("row count", 0, "row of mat graphs", this, SLOT(UpdateRow()));
  row_prop_->setMin(1);
  row_prop_->setMax(10);
  col_prop_ = new rviz::IntProperty("col count", 0, "col of mat graphs", this, SLOT(UpdateCol()));
  col_prop_->setMin(1);
  col_prop_->setMax(10);
}

void MatrixDisplay::UpdateRow()
{
  int const old_row = fields_prop_.rows();
  int const new_row = row_prop_->getInt();
  int const col = fields_prop_.cols();
  // 增加
  if (old_row < new_row)
  {
    fields_prop_.conservativeResize(new_row, col);
    for (int i = old_row; i < new_row; ++i)
    {
      for (int j = 0; j < fields_prop_.cols(); ++j)
      {
        fields_prop_(i, j) = new rviz::StringProperty(QString("field-%1-%2").arg(i).arg(j), "", "matrix plot field", this);
        connect(fields_prop_(i, j), &Property::changed, [this, i, j]()
                { this->UpdateFieldName(i, j); });
      }
    }
  }
  else // 删除
  {
    for (int i = new_row; i < old_row; ++i)
    {
      for (int j = 0; j < col; ++j)
      {
        this->takeChild(fields_prop_(i, j));
        delete fields_prop_(i, j);
      }
    }
    fields_prop_.conservativeResize(new_row, col); // 删除操作
  }
}
void MatrixDisplay::UpdateCol()
{

  int const old_col = fields_prop_.cols();
  int const new_col = col_prop_->getInt();
  int const row = fields_prop_.rows();
  // 增加
  if (old_col < new_col)
  {
    fields_prop_.conservativeResize(row, new_col);
    for (int i = 0; i < row; ++i)
    {
      for (int j = old_col; j < new_col; ++j)
      {
        fields_prop_(i, j) = new rviz::StringProperty(QString("field-%1-%2").arg(i).arg(j), "", "matrix plot field", this);
        connect(fields_prop_(i, j), &Property::changed, [this, i, j]()
                { this->UpdateFieldName(i, j); });
      }
    }
  }
  else // 删除
  {
    for (int i = 0; i < row; ++i)
    {
      for (int j = new_col; j < old_col; ++j)
      {
        this->takeChild(fields_prop_(i, j));
        delete fields_prop_(i, j);
      }
    }
    fields_prop_.conservativeResize(row, new_col); // 删除操作
  }
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

MatrixDisplay::~MatrixDisplay()
{
  if (initialized())
  {
    delete view_;
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