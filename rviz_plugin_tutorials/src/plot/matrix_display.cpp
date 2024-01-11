#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/visualization_manager.h>

#include "plot/matrix_display.h"
#include "plot/matrix_widget.h"
#include "protobuf_helper.h"

MatrixDisplay::MatrixDisplay()
{
  InitPersons();
  view_ = new MatrixWidget();
  view_->setDisplaySync(this);
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

void MatrixDisplay::AddSeries(QString const &name, QStringList const &field_names)
{
  view_->AddSeries(name, field_names);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(MatrixDisplay, rviz::Display)