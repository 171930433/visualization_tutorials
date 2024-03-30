#pragma once

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>

#include <rviz_common/display.hpp>

#include "display_sync_base.h"
#include <deque>

namespace rviz_common {
namespace properties {

class IntProperty;
class EnumProperty;
class BoolProperty;
class GroupProperty;
class ColorProperty;

} // namespace properties
} // namespace rviz_common

class PrecisionWidget;

class PrecisionDisplay : public DisplaySyncBase {
  Q_OBJECT
public:
  PrecisionDisplay();
  ~PrecisionDisplay() override;

  // Overrides from Display
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;
  void onInitialize() override;
  void update(float dt, float ros_dt) override;
private Q_SLOTS:

private:
  PrecisionWidget *view_ = nullptr;

private:
  rviz_common::properties::EnumProperty *plot_type_prop_;
};
