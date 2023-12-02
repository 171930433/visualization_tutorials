/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// %EndTag(INCLUDES)%

visualization_msgs::Marker CreateArrow()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "arrow";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -10;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  // 长 宽 高
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  // color
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker CreateCube()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "cube";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -9;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  // 长 宽 高
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  // color
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker CreateSphere()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "sphere";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -8;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  // 长 宽 高
  marker.scale.x = 0.25;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  // color
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker CreateCylinder()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "cylinder";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -7;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  // 长 宽 高
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 1;
  // color
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker CreateLineStrip()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "line strip";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -6;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  // points
  geometry_msgs::Point pt;
  marker.points.reserve(10);
  pt.x = 0, pt.y = -2, pt.z = -1, marker.points.push_back(pt);
  pt.x = 0, pt.y = -1, pt.z = -0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 0, pt.z = 0, marker.points.push_back(pt);
  pt.x = 0, pt.y = 1, pt.z = 0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 2, pt.z = 1, marker.points.push_back(pt);
  // 线段宽度
  marker.scale.x = 0.1;

  // color
  marker.color.r = 0;
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker CreateLineList()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "line list";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -5;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  // points
  geometry_msgs::Point pt;
  marker.points.reserve(10);
  pt.x = 0, pt.y = -2, pt.z = -1, marker.points.push_back(pt);
  pt.x = 0, pt.y = -2, pt.z = 1, marker.points.push_back(pt);
  pt.x = 0, pt.y = -1, pt.z = -0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = -1, pt.z = 0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 0, pt.z = 0, marker.points.push_back(pt);
  pt.x = 0, pt.y = 0, pt.z = 0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 1, pt.z = -0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 1, pt.z = 0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 2, pt.z = -1, marker.points.push_back(pt);
  pt.x = 0, pt.y = 2, pt.z = 1, marker.points.push_back(pt);
  // 线段宽度
  marker.scale.x = 0.1;

  // color
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker CreateCuberList()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "cube list ";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -4;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  // points
  geometry_msgs::Point pt;
  marker.points.reserve(10);
  pt.x = 0, pt.y = -2, pt.z = -1, marker.points.push_back(pt);
  pt.x = 0, pt.y = -1, pt.z = -0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 0, pt.z = 0, marker.points.push_back(pt);
  pt.x = 0, pt.y = 1, pt.z = 0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 2, pt.z = 1, marker.points.push_back(pt);
  // 线段宽度
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // color
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker CreateSphereList()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "sphere list ";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -3;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  // points
  geometry_msgs::Point pt;
  marker.points.reserve(10);
  pt.x = 0, pt.y = -2, pt.z = -1, marker.points.push_back(pt);
  pt.x = 0, pt.y = -1, pt.z = -0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 0, pt.z = 0, marker.points.push_back(pt);
  pt.x = 0, pt.y = 1, pt.z = 0.5, marker.points.push_back(pt);
  pt.x = 0, pt.y = 2, pt.z = 1, marker.points.push_back(pt);
  // 线段宽度
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // color
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker CreatePoints()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "points ";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -2;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  // points
  geometry_msgs::Point pt;
  float offset = 0.2;
  marker.points.reserve(50);
  for (int i = 0; i < 5; i++)
  {
    pt.x = (-2 + i) * offset;
    for (int j = 0; j < 5; j++)
    {
      pt.y = (-2 + j) * offset;
      pt.z = 0;
      marker.points.push_back(pt);
    }
  }

  // 点的长宽
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;

  // color
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker CreateViewOrientedText ()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "ViewOrientedText ";
  marker.id = 0;
  //
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  // pose
  marker.pose.position.x = -2;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0.5;

  // test
  marker.text = "points";
  marker.scale.z = 0.5;

  // color
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 1;
  // 持续时间
  marker.lifetime = ros::Duration();

  return marker;
}

// %Tag(INIT)%
int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);

  while (ros::ok())
  {

    // %Tag(PUBLISH)%
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      ros::Duration(1.0).sleep();
    }

    marker_pub.publish(CreateArrow());
    marker_pub.publish(CreateCube());
    marker_pub.publish(CreateSphere());
    marker_pub.publish(CreateCylinder());
    marker_pub.publish(CreateLineStrip());
    marker_pub.publish(CreateLineList());
    marker_pub.publish(CreateCuberList());
    marker_pub.publish(CreateSphereList());
    marker_pub.publish(CreatePoints());
    marker_pub.publish(CreateViewOrientedText());

    r.sleep();
  }
  // %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
