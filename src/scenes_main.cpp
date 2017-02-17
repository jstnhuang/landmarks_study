#include <iostream>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "boost/algorithm/string.hpp"
#include "mongodb_store/message_store.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_perception/conversions.h"
#include "rapid_perception/rgbd.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "set_scenes_main");
  ros::NodeHandle nh;
  tf::TransformListener tf_listener;
  mongodb_store::MessageStoreProxy db(nh, "scenes", "landmarks_study");
  ros::Publisher scene_pub =
      nh.advertise<sensor_msgs::PointCloud2>("scene", 1, true);
  while (ros::ok()) {
    std::cout << std::endl;
    std::cout << "list: See point cloud names." << std::endl;
    std::cout << "load <name>: Load a point cloud." << std::endl;
    std::cout << "save <name>: Save the cloud_in as <name>." << std::endl;
    std::cout << "exit: Exit this application." << std::endl;
    std::cout << "Enter a command: ";

    std::string input("");
    std::getline(std::cin, input);
    std::vector<std::string> tokens;
    boost::split(tokens, input, boost::is_space());
    if (tokens.size() == 0) {
      continue;
    }
    std::string command(tokens[0]);
    std::string name("");
    if (tokens.size() == 2) {
      name = tokens[1];
    }

    if (command == "list") {
      std::cout << "Not implemented." << std::endl;
      // std::vector<std_msgs::String::Ptr> results;
      // std::cout << std::endl;
      // for (size_t i = 0; i < results.size(); ++i) {
      //  std::cout << results[i]->data << std::endl;
      //}
    } else if (command == "load") {
      std::vector<sensor_msgs::PointCloud2::Ptr> results;
      bool success = db.queryNamed(name, results);
      if (!success || results.size() == 0) {
        ROS_ERROR("No cloud with name %s", name.c_str());
        continue;
      }
      scene_pub.publish(results[0]);
    } else if (command == "save") {
      int num_clouds = 5;
      ros::param::param<int>("num_clouds", num_clouds, 5);
      sensor_msgs::PointCloud2::Ptr cloud = rapid::perception::RosFromPcl(
          rapid::perception::GetSmoothedKinectCloud("cloud_in", num_clouds));

      sensor_msgs::PointCloud2 cloud_base;
      pcl_ros::transformPointCloud("base_link", *cloud, cloud_base,
                                   tf_listener);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(cloud_base, *pcl_cloud);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_cropped(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::CropBox<pcl::PointXYZRGB> crop;
      crop.setInputCloud(pcl_cloud);
      Eigen::Vector4f min_pt;
      min_pt << 0, -0.75, 0.2, 1;
      Eigen::Vector4f max_pt;
      max_pt << 1.2, 0.75, 1.7, 1;
      crop.setMin(min_pt);
      crop.setMax(max_pt);
      crop.filter(*pcl_cloud_cropped);

      std::cout << "Saved cloud with " << pcl_cloud_cropped->size()
                << " points." << std::endl;

      sensor_msgs::PointCloud2 final_cloud;
      pcl::toROSMsg(*pcl_cloud_cropped, final_cloud);
      scene_pub.publish(final_cloud);
      db.updateNamed(name, final_cloud, true);
    } else if (input == "exit") {
      return 0;
    } else {
      std::cout << "Unknown command: " << command << std::endl;
    }
  }
  return 0;
}
