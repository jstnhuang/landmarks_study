#include "ros/ros.h"

#include <string>
#include <vector>

#include "mongodb_store/message_store.h"
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
  mongodb_store::MessageStoreProxy participant_db(nh, "participants",
                                                  "landmarks_study");
  mongodb_store::MessageStoreProxy landmarks_db(nh, "landmarks",
                                                "landmarks_study");
  mongodb_store::MessageStoreProxy scenes_db(nh, "scenes", "landmarks_study");

  rapid::perception::Box3DRoiServer roi("roi");

  // Build publishers
  ros::Publisher scene_pub =
      nh.advertise<sensor_msgs::PointCloud2>("scene", 1, true);
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

  study::Experiment experiment(participant_db, landmarks_db, scenes_db, roi,
                               scene_pub, alignment_pub, output_pub,
                               pose_estimator, task_list);
  ros::Subscriber action_sub = nh.subscribe(
      "user_actions", 10, &study::Experiment::ProcessAction, &experiment);
  ros::spin();
  return 0;
}
