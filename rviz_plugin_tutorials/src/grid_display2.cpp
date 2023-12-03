/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <stdint.h>

#include <boost/bind/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
// #include <rviz/ogre_helpers/grid.h>
#include <rviz/properties/parse_color.h>
#include <rviz/properties/property.h>
#include <rviz/selection/selection_manager.h>

#include "grid_view2.h"
#include "grid_display2.h"

// #include <rviz/view_manager.h>
// #include <rviz/default_plugin/view_controllers/fixed_orientation_ortho_view_controller.h>

namespace rviz
{
GridDisplay2::GridDisplay2() : Display()
{
  frame_property_ =
      new TfFrameProperty("Reference Frame", TfFrameProperty::FIXED_FRAME_STRING,
                          "The TF frame this grid will use for its origin.", this, nullptr, true);

  cell_count_property_ =
      new IntProperty("Plane Cell Count", 10, "The number of cells to draw in the plane of the grid.",
                      this, &GridDisplay2::updateCellCount);
  cell_count_property_->setMin(1);

  height_property_ = new IntProperty("Normal Cell Count", 0,
                                     "The number of cells to draw along the normal vector of the grid. "
                                     " Setting to anything but 0 makes the grid 3D.",
                                     this, &GridDisplay2::updateHeight);
  height_property_->setMin(0);

  cell_size_property_ =
      new FloatProperty("Cell Size", 1.0f, "The length, in meters, of the side of each cell.", this,
                        &GridDisplay2::updateCellSize);
  cell_size_property_->setMin(0.0001);

  style_property_ =
      new EnumProperty("Line Style", "Lines", "The rendering operation to use to draw the grid lines.",
                       this, &GridDisplay2::updateStyle);
  style_property_->addOption("Lines", Grid2::Lines);
  style_property_->addOption("Billboards", Grid2::Billboards);

  line_width_property_ =
      new FloatProperty("Line Width", 0.03, "The width, in meters, of each grid line.", style_property_,
                        &GridDisplay2::updateLineWidth, this);
  line_width_property_->setMin(0.001);
  line_width_property_->hide();

  color_property_ = new ColorProperty("Color", Qt::gray, "The color of the grid lines.", this,
                                      &GridDisplay2::updateColor);
  alpha_property_ =
      new FloatProperty("Alpha", 0.5f, "The amount of transparency to apply to the grid lines.", this,
                        &GridDisplay2::updateColor);
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  plane_property_ = new EnumProperty("Plane", "XY", "The plane to draw the grid along.", this,
                                     &GridDisplay2::updatePlane);
  plane_property_->addOption("XY", XY);
  plane_property_->addOption("XZ", XZ);
  plane_property_->addOption("YZ", YZ);

  offset_property_ = new VectorProperty(
      "Offset", Ogre::Vector3::ZERO,
      "Allows you to offset the grid from the origin of the reference frame.  In meters.", this,
      &GridDisplay2::updateOffset);
}

GridDisplay2::~GridDisplay2()
{
  if (initialized())
  {
    delete grid_;
  }
}

void GridDisplay2::onInitialize()
{
  QColor color = color_property_->getColor();
  color.setAlphaF(alpha_property_->getFloat());

  frame_property_->setFrameManager(context_->getFrameManager());
  grid_ = new Grid2(scene_manager_, scene_node_, (Grid2::Style)style_property_->getOptionInt(),
                   cell_count_property_->getInt(), cell_size_property_->getFloat(),
                   line_width_property_->getFloat(), qtToOgre(color));

  grid_->getSceneNode()->setVisible(false);
  updatePlane();
}

void GridDisplay2::update(float /*dt*/, float /*ros_dt*/)
{
  QString qframe = frame_property_->getFrame();
  std::string frame = qframe.toStdString();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (context_->getFrameManager()->getTransform(frame, ros::Time(), position, orientation))
  {
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }
  else
  {
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(frame, ros::Time(), error))
    {
      setStatus(StatusProperty::Error, "Transform", QString::fromStdString(error));
    }
    else
    {
      setStatus(StatusProperty::Error, "Transform",
                "Could not transform from [" + qframe + "] to [" + fixed_frame_ + "]");
    }
  }

  // if (scene_manager_->getCurrentViewport())
  // {
  //   if (FixedOrientationOrthoViewController* vc =
  //           qobject_cast<FixedOrientationOrthoViewController*>(context_->getViewManager()->getCurrent()))
  //   {
  //     auto* vp = scene_manager_->getCurrentViewport();
  //     // 实际像素宽度
  //     float act_width = vp->getActualWidth();
  //     float act_height = vp->getActualHeight();
  //     float act_top = vp->getActualTop();
  //     float act_left = vp->getActualLeft();
  //     auto mat = vp->getCamera()->getViewMatrix();
  //     auto pos = vp->getCamera()->getPosition();
  //     auto rea_pos = vp->getCamera()->getRealPosition();
  //     double xx = mat[0][3], yy = mat[1][3], zz = mat[2][3];
  //     // scale
  //     float scale = vc->getScale();
  //     ROS_INFO("act_width=%lf act_height=%lf scale = %lf top = %lf left = %lf, ", act_width, act_height, scale, act_top, act_left);
  //     ROS_INFO("xx=%lf yy=%lf zz = %lf", xx, yy, zz);
  //     std::cout << "pos = " << pos <<"\n";
  //     std::cout << "rea_pos = " << rea_pos <<"\n";
  //     float cell_size = (act_width >= act_height ? act_width / scale : act_height / scale);
  //     if (cell_size != cell_count_property_->getInt())
  //     {
  //       cell_count_property_->setInt(cell_size);
  //     }
  //   }

  //   // ROS_INFO("width=%lf height=%lf scale=%lf", width,height,scale);
  // }
}

void GridDisplay2::updateColor()
{
  QColor color = color_property_->getColor();
  color.setAlphaF(alpha_property_->getFloat());
  grid_->setColor(qtToOgre(color));
  context_->queueRender();
}

void GridDisplay2::updateCellSize()
{
  grid_->setCellLength(cell_size_property_->getFloat());
  context_->queueRender();
}

void GridDisplay2::updateCellCount()
{
  grid_->setCellCount(cell_count_property_->getInt());
  context_->queueRender();
}

void GridDisplay2::updateLineWidth()
{
  grid_->setLineWidth(line_width_property_->getFloat());
  context_->queueRender();
}

void GridDisplay2::updateHeight()
{
  grid_->setHeight(height_property_->getInt());
  context_->queueRender();
}

void GridDisplay2::updateStyle()
{
  Grid2::Style style = (Grid2::Style)style_property_->getOptionInt();
  grid_->setStyle(style);

  switch (style)
  {
  case Grid2::Billboards:
    line_width_property_->show();
    break;
  case Grid2::Lines:
  default:
    line_width_property_->hide();
    break;
  }
  context_->queueRender();
}

void GridDisplay2::updateOffset()
{
  grid_->getSceneNode()->setPosition(offset_property_->getVector());
  context_->queueRender();
}

void GridDisplay2::updatePlane()
{
  Ogre::Quaternion orient;
  switch ((Plane)plane_property_->getOptionInt())
  {
  case XZ:
    orient = Ogre::Quaternion(1, 0, 0, 0);
    break;
  case YZ:
    orient = Ogre::Quaternion(Ogre::Vector3(0, -1, 0), Ogre::Vector3(0, 0, 1), Ogre::Vector3(1, 0, 0));
    break;
  case XY:
  default:
    orient = Ogre::Quaternion(Ogre::Vector3(1, 0, 0), Ogre::Vector3(0, 0, -1), Ogre::Vector3(0, 1, 0));
    break;
  }
  grid_->getSceneNode()->setOrientation(orient);

  context_->queueRender();
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::GridDisplay2, rviz::Display)
