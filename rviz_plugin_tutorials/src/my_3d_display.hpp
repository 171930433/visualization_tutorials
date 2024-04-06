#pragma once

#include <QObject>
#include <atomic>
#include <rviz_common/display.hpp>

namespace rviz_common {
class Display;
class RenderPanel;
class VisualizationManager;
class ViewManager;

} // namespace rviz_common

class My3dDisplay : public rviz_common::Display {
public:
  My3dDisplay();

  void initialize(rviz_common::DisplayContext *context) override;

protected:
  void setupRenderPanel();
  void update(float wall_dt, float ros_dt) override;
  bool updateCamera();

protected:
  std::shared_ptr<rviz_common::VisualizationManager> manager_;
  std::shared_ptr<rviz_common::RenderPanel> render_panel_;
  std::shared_ptr<rviz_common::ViewManager> view_manager_;
  std::atomic_bool inited_ = {false};
};