#pragma once

// #include <rviz/properties/color_property.h>
// #include <rviz/properties/float_property.h>
// #include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>

// #include <rviz/properties/vector_property.h>
// #include <rviz/properties/enum_property.h>
// #include <rviz/properties/tf_frame_property.h>
#include <rviz/display.h>

namespace rviz
{
    class IntProperty;
}

class TrajectoryWidget;

class TrajectoryDisplay : public rviz::Display
{
    Q_OBJECT
public:
    TrajectoryDisplay();
    ~TrajectoryDisplay() override;

    // 需要在 Initialize 之前调用,以确定在onInitialize 绑定有效
    void setView(TrajectoryWidget *view) { view_ = view; }

    // Overrides from Display
    void onInitialize() override;
    void update(float dt, float ros_dt) override;

private Q_SLOTS:
    void UpdateInterval();
    void UpdateRange();
private:
    TrajectoryWidget *view_ = nullptr;

    rviz::IntProperty *main_interval_;
    rviz::IntProperty *sub_range_;
};
