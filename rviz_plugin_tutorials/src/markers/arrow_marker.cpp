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

#include "markers/arrow_marker.h"
#include "markers/my_marker_selection_handler.h"


#include <rviz/ogre_helpers/arrow.h>
#include <rviz/display_context.h>
#include <rviz/default_plugin/markers/marker_selection_handler.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace rviz {

void MyArrowMarker::onNewMessage(const MarkerConstPtr &old_message, const MarkerConstPtr &new_message) {
  if (!arrow_) {
    arrow_ = new Arrow(context_->getSceneManager(), child_scene_node_);
    setDefaultProportions();
    auto* handler = new MyMarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id), context_);
    handler->setDisplaySceneNode(scene_node_);
    handler_.reset(handler);
    handler_->addTrackedObjects(arrow_->getSceneNode());
  }

  ArrowMarker::onNewMessage(old_message, new_message);
}

} // namespace rviz
