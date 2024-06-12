#include "visual/my_pointcloud.hpp"

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <rviz/display.h>

void MyPointCloud::initialize(rviz::Display *display, rviz::DisplayContext *context) {
  pc_common_ = std::make_shared<rviz::PointCloudCommon>(display);
  pc_common_->initialize(context, display->getSceneNode());

  shape_property_ = new rviz::BoolProperty("MyPointCloud", true, "", display);
  // connect(shape_property_, &rviz::Property::changed, this, &MyPointCloud::ColorChanged);

  Eigen::Vector3f raw_pt1{0, 0.8, 0};
  Eigen::Vector3f raw_pt2{0, 1, 0};
  pts_ = {raw_pt1, raw_pt2};

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto const &pt : pts_) {
    cloud->points.push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
  }

  // //
  sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);

  // // points

  pcl::toROSMsg(*cloud, *output);
  output->header.frame_id = "map";
  output->header.stamp = ros::Time::now();
  output->header.seq = 1;

  std::cout << *output << "\n";

  pc_common_->addMessage(output);
}

void MyPointCloud::update(float wall_dt, float ros_dt) {
  //
  pc_common_->update(wall_dt, ros_dt);
}