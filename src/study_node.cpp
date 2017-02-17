#include "ros/ros.h"

#include <string>
#include <vector>

#include "rapid_db/name_db.hpp"
#include "rapid_msgs/GetStaticCloud.h"
#include "rapid_msgs/ListStaticClouds.h"
#include "rapid_msgs/RemoveStaticCloud.h"
#include "rapid_msgs/SaveStaticCloud.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/random_heat_mapper.h"
#include "sensor_msgs/PointCloud2.h"

#include "landmarks_study/experiment.h"
#include "landmarks_study/UserAction.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "study_node");
  ros::NodeHandle nh;

  // Build databases
  rapid::db::NameDb participant_db(nh, "landmarks_study", "participants");
  rapid::db::NameDb landmark_db(nh, "landmarks_study", "landmarks");
  rapid::db::NameDb scene_db(nh, "landmarks_study", "scenes");

  rapid::perception::Box3DRoiServer roi("roi");
  roi.set_base_frame("base_link");

  // Build publishers
  ros::Publisher scene_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/scene", 1, true);
  ros::Publisher alignment_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/alignment", 1, true);
  ros::Publisher output_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/output", 1, true);

  // Build CustomLandmarks
  rapid::perception::RandomHeatMapper* heat_mapper =
      new rapid::perception::RandomHeatMapper();
  heat_mapper->set_name("random");
  rapid::perception::PoseEstimator pose_estimator(heat_mapper);
  pose_estimator.set_alignment_publisher(alignment_pub);
  pose_estimator.set_output_publisher(output_pub);

  // Build experiment
  std::vector<std::string> task_list;
  nh.getParam("experiment_tasks", task_list);

  study::Experiment experiment(&participant_db, &landmark_db, &scene_db, roi,
                               scene_pub, alignment_pub, output_pub,
                               pose_estimator, task_list);
  ros::Subscriber action_sub = nh.subscribe(
      "user_actions", 10, &study::Experiment::ProcessAction, &experiment);
  ros::spin();
  return 0;
}
