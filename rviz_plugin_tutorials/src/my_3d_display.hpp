#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>

// view
#include <rviz/default_plugin/view_controllers/fixed_orientation_ortho_view_controller.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/viewport_mouse_event.h>

namespace rviz {
class DisplayGroupVisibilityProperty;

class MyFixedOrientationOrthoViewController : public FixedOrientationOrthoViewController {
public:
  MyFixedOrientationOrthoViewController() : FixedOrientationOrthoViewController() {
    z_property_ = new FloatProperty("Z", 0, "Z component of camera position.", this);
    // plane_property_ = new EnumProperty("Plane", "XY", "The plane to draw the grid along.", this, SLOT(updatePlane()));
    plane_property_ = new EnumProperty("Plane", "XY", "The plane to draw the grid along.", nullptr);
    plane_property_->addOption("XY", 0);
    plane_property_->addOption("XZ", 1);
    plane_property_->addOption("YZ", 2);
    plane_property_->setReadOnly(true);

    this->addChild(plane_property_, 0);
  }

  void handleMouseEvent(ViewportMouseEvent &event) override {
    event.modifiers.setFlag(Qt::ShiftModifier); // 强制shift锁定
    FixedOrientationOrthoViewController::handleMouseEvent(event);
  }

protected:
  FloatProperty *z_property_;
  EnumProperty* plane_property_;
};
} // namespace rviz

class My3dDisplay : public rviz::Display {
public:
  My3dDisplay();
  ~My3dDisplay();

  void onInitialize() override;

protected:
  void setupRenderPanel();
  void update(float wall_dt, float ros_dt) override;
  // bool updateCamera();

protected:
  std::shared_ptr<rviz::VisualizationManager> manager_;
  std::shared_ptr<rviz::RenderPanel> render_panel_;
  std::shared_ptr<rviz::ViewManager> view_manager_;
  std::atomic_bool inited_ = {false};

  // 最佳自定义图元绘制
protected:
protected:
  uint32_t vis_bit_;
  rviz::DisplayGroupVisibilityProperty *visibility_property_;
  rviz::FixedOrientationOrthoViewController *top_down_view_;
};