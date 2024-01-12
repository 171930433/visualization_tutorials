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

  // for (int i = 0; i < 3; ++i)
  // {
  //   field_prop_[i] = new rviz::StringProperty(QString("field-%1").arg(i), "", "vec3 plot field", this);
  //   connect(field_prop_[i], &Property::changed, [this, i]()
  //           { this->UpdateFieldName(i); });
  // }
}

void MatrixDisplay::UpdateFieldName(int const row, int const col)
{
  QString const &field_name = fields_prop_(row, col)->getString();
  view_->UpdateFieldName(row, col, field_name);
}

void MatrixDisplay::CreateMatrixPlot(QString const &name, MatrixXQString const &field_names)
{
  view_->CreatePlot(name, field_names);
  // 创建property
  fields_prop_ = MatrixXQStringProp(field_names.rows(), field_names.cols());
  for (int i = 0; i < field_names.rows(); ++i)
  {
    for (int j = 0; j < field_names.cols(); ++j)
    {
      fields_prop_(i, j) = new rviz::StringProperty(QString("field-%1-%2").arg(i).arg(j), "", "matrix plot field", this);
      connect(fields_prop_(i, j), &Property::changed, [this, i, j]()
              { this->UpdateFieldName(i, j); });
      // 赋值
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