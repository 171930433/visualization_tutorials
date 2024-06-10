#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/display_context.h>
#include <rviz/visualization_manager.h>


class My3dDisplay : public rviz::Display {
public:
  My3dDisplay();

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
};