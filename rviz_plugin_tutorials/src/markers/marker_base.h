#pragma once

#include <rviz/default_plugin/markers/marker_base.h>
#include <rviz/display.h>

namespace rviz {

class MyMarkerBase : public MarkerBase {
public:
  using MarkerBase::MarkerBase;
  // 只可以在initialize函数中构造
  MyMarkerBase(Display *display, DisplayContext *context);
};

} // namespace rviz