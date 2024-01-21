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
#include <deque>
namespace rviz
{
  class IntProperty;
  class EnumProperty;
  class BoolProperty;
  class GroupProperty;
  class ColorProperty;
}

class PrecisionWidget;

class PrecisionDisplay : public DisplaySyncBase
{
  Q_OBJECT
public:
  PrecisionDisplay();
  ~PrecisionDisplay() override;

  // Overrides from Display
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;
  void onInitialize() override;
  void update(float dt, float ros_dt) override;
private Q_SLOTS:

private:
  PrecisionWidget *view_ = nullptr;

private:
  rviz::EnumProperty *plot_type_prop_;
};
