#include "markers/marker_base.h"

#include <rviz/default_plugin/marker_display.h>
#include <rviz/default_plugin/markers/marker_selection_handler.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/selection/selection_manager.h>

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreSubEntity.h>

namespace rviz {
MyMarkerBase::MyMarkerBase(Display *display, DisplayContext *context)
    : MarkerBase(nullptr, context, display->getSceneNode()) {}

} // namespace rviz