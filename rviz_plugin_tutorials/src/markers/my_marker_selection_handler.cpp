/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <rviz/default_plugin/interactive_markers/interactive_marker_control.h>
#include <rviz/default_plugin/marker_display.h>
#include <rviz/default_plugin/markers/marker_base.h>
#include <rviz/properties/property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/vector_property.h>

#include "markers/arrow_marker.h"
#include "markers/my_marker_selection_handler.h"

namespace rviz {

MyMarkerSelectionHandler::MyMarkerSelectionHandler(const MarkerBase *marker, MarkerID id, DisplayContext *context)
    : MarkerSelectionHandler(marker, id, context) {
  my_marker_ = marker;
}

void MyMarkerSelectionHandler::createProperties(const Picked &obj, Property *parent_property) {
  MarkerSelectionHandler::createProperties(obj, parent_property);

  auto any_cbk = Ogre::any_cast<rviz::Display *>(display_scene_node_->getUserObjectBindings().getUserAny());
  any_cbk->setTopic( QString::fromStdString(getStringID()), "");
}

MarkerBase *
createMarker2(int marker_type, MarkerDisplay *owner, DisplayContext *context, Ogre::SceneNode *parent_node) {
  switch (marker_type) {
  case visualization_msgs::Marker::ARROW:
    return new rviz::MyArrowMarker(owner, context, parent_node);
  default:
    return createMarker(marker_type, owner, context, parent_node);
  }
  return nullptr;
}

} // end namespace rviz
