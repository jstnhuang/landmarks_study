#include "landmarks_study/experiment.h"

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/time.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rapid_db/name_db.hpp"
#include "rapid_msgs/Roi3D.h"
#include "rapid_msgs/StaticCloud.h"
#include "rapid_perception/box3d_roi_server.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/random_heat_mapper.h"

#include "landmarks_study/Participant.h"
#include "landmarks_study/Task.h"
#include "landmarks_study/UserAction.h"
#include "landmarks_study/experiment_constants.h"

using landmarks_study::Participant;
using landmarks_study::Task;
using landmarks_study::UserAction;
using pcl::PointCloud;
using pcl::PointXYZRGB;
using rapid_msgs::Roi3D;
using std::string;

namespace study {
Experiment::Experiment(rapid::db::NameDb* participant_db,
                       rapid::db::NameDb* landmark_db,
                       rapid::db::NameDb* scene_db,
                       const rapid::perception::Box3DRoiServer& roi,
                       const ros::Publisher& scene_pub,
                       const ros::Publisher& alignment_pub,
                       const ros::Publisher& output_pub,
                       const rapid::perception::PoseEstimator& pose_estimator,
                       const std::vector<string>& task_list)
    : participant_db_(participant_db),
      landmark_db_(landmark_db),
      scene_db_(scene_db),
      roi_(roi),
      scene_pub_(scene_pub),
      alignment_pub_(alignment_pub),
      output_pub_(output_pub),
      pose_estimator_(pose_estimator),
      task_list_(task_list) {}

void Experiment::ProcessAction(const landmarks_study::UserAction& action) {
  const string& task_name =
      TaskName(action.participant_name, action.task_number);
  if (task_name == kEndTask) {
    ClearTestVisualization();
    return;
  }

  if (action.action == landmarks_study::UserAction::LOAD) {
    Load(action, task_name);
  } else if (action.action == landmarks_study::UserAction::EDIT) {
    Edit(action, task_name);
  } else if (action.action == landmarks_study::UserAction::TEST) {
    Test(action, task_name);
  } else if (action.action == landmarks_study::UserAction::SAVE) {
    Save(action, task_name);
  } else {
    ROS_ERROR("Unknown action: \"%s\"", action.action.c_str());
  }
}

void Experiment::Load(const UserAction& action, const string& task_name) {
  ClearTestVisualization();

  // Load participant if already in the DB.
  Participant participant;
  if (!participant_db_->Get(action.participant_name, &participant)) {
    ROS_INFO("Participant \"%s\" not in DB, inserting",
             action.participant_name.c_str());
    participant.name = action.participant_name;
    participant.tasks.clear();
  }

  // Load task if already in the DB.
  Task* task = GetTask(participant, task_name);
  if (task == NULL) {
    ROS_INFO("Task \"%s\" not in DB, inserting", task_name.c_str());
    Task t;
    t.name = task_name;
    participant.tasks.push_back(t);
    task = &participant.tasks[participant.tasks.size() - 1];
  }
  ROS_INFO("%ld events recorded for task \"%s\"", task->actions.size(),
           task->name.c_str());

  task->actions.push_back(action);
  SaveParticipant(participant);

  // Publish scene
  sensor_msgs::PointCloud2 scene;
  scene_db_->Get(TrainSceneName(task_name), &scene);
  scene_pub_.publish(scene);

  ROS_INFO("Loaded participant \"%s\", task \"%s\"", participant.name.c_str(),
           task->name.c_str());
}

void Experiment::Edit(const UserAction& action, const string& task_name) {
  ClearTestVisualization();

  Participant participant;
  if (!participant_db_->Get(action.participant_name, &participant)) {
    ROS_ERROR("Error editing participant \"%s\"",
              action.participant_name.c_str());
    return;
  }
  Task* task = GetTask(participant, task_name);
  if (task == NULL) {
    ROS_ERROR("Error editing task \"%s\" for participant \"%s\"",
              task_name.c_str(), action.participant_name.c_str());
    return;
  }
  task->actions.push_back(action);
  SaveParticipant(participant);

  const Roi3D& roi = task->roi;

  if (roi.dimensions.x == 0 || roi.dimensions.y == 0 || roi.dimensions.z == 0) {
    roi_.Start();
  } else {
    roi_.Start(roi.transform.translation.x, roi.transform.translation.y,
               roi.transform.translation.z, roi.dimensions.x, roi.dimensions.y,
               roi.dimensions.z);
  }

  // Publish scene
  sensor_msgs::PointCloud2 scene;
  scene_db_->Get(TrainSceneName(task_name), &scene);
  scene_pub_.publish(scene);

  ROS_INFO("Editing ROI for participant \"%s\", task \"%s\"",
           action.participant_name.c_str(), task_name.c_str());
}

void Experiment::Test(const UserAction& action, const string& task_name) {
  Participant participant;
  if (!participant_db_->Get(action.participant_name, &participant)) {
    ROS_ERROR("Error testing participant \"%s\"",
              action.participant_name.c_str());
    return;
  }
  Task* task = GetTask(participant, task_name);
  if (task == NULL) {
    ROS_ERROR("Error testing task \"%s\" for participant \"%s\"",
              task_name.c_str(), action.participant_name.c_str());
    return;
  }
  task->actions.push_back(action);

  // Get the landmark.
  sensor_msgs::PointCloud2 scene;
  scene_db_->Get(TrainSceneName(task_name), &scene);
  task->roi = roi_.roi();
  rapid_msgs::StaticCloud landmark;
  ComputeLandmark(task->roi, scene, &landmark);

  // Save the ROI.
  SaveParticipant(participant);

  // Stop publishing the box
  roi_.Stop();

  // Publish the test scene.
  sensor_msgs::PointCloud2 test_scene;
  scene_db_->Get(TestSceneName(task_name), &test_scene);
  scene_pub_.publish(test_scene);

  ROS_INFO("Testing landmark for participant \"%s\", task \"%s\"",
           participant.name.c_str(), task->name.c_str());
  pcl::StopWatch watch;
  watch.reset();

  // Run CustomLandmarks
  PointCloud<PointXYZRGB>::Ptr landmark_cloud(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(landmark.cloud, *landmark_cloud);
  PointCloud<PointXYZRGB>::Ptr test_scene_pcl(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(test_scene, *test_scene_pcl);

  // Downsample
  PointCloud<PointXYZRGB>::Ptr landmark_cloud_sampled(
      new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr test_scene_sampled(new PointCloud<PointXYZRGB>);
  double leaf_size = 0.01;
  ros::param::param<double>("leaf_size", leaf_size, 0.01);
  pcl::VoxelGrid<PointXYZRGB> vox;
  vox.setLeafSize(leaf_size, leaf_size, leaf_size);
  vox.setInputCloud(landmark_cloud);
  vox.filter(*landmark_cloud_sampled);
  vox.setInputCloud(test_scene_pcl);
  vox.filter(*test_scene_sampled);

  pose_estimator_.set_object(landmark_cloud_sampled);
  pose_estimator_.set_roi(task->roi);
  pose_estimator_.set_scene(test_scene_sampled);
  UpdateParams();
  std::vector<rapid::perception::PoseEstimationMatch> matches;
  pose_estimator_.Find(&matches);

  ClearAlignmentVisualization();

  ROS_INFO("Testing took %f seconds for participant \"%s\", task \"%s\"",
           watch.getTimeSeconds(), participant.name.c_str(),
           task->name.c_str());
}

void Experiment::Save(const UserAction& action, const string& task_name) {
  Participant participant;
  if (!participant_db_->Get(action.participant_name, &participant)) {
    ROS_ERROR("Error saving participant \"%s\"",
              action.participant_name.c_str());
    return;
  }
  Task* task = GetTask(participant, task_name);
  if (task == NULL) {
    ROS_ERROR("Error saving task \"%s\" for participant \"%s\"",
              task_name.c_str(), action.participant_name.c_str());
    return;
  }
  task->actions.push_back(action);

  task->roi = roi_.roi();
  SaveParticipant(participant);
}

bool Experiment::SaveParticipant(const Participant& participant) {
  Participant p;
  if (participant_db_->Get(participant.name, &p)) {
    ROS_INFO("Updating participant \"%s\"", participant.name.c_str());
    bool success = participant_db_->Update(participant.name, participant);
    if (!success) {
      ROS_ERROR("Failed!");
    }
  } else {
    ROS_INFO("Inserting participant \"%s\"", participant.name.c_str());
    participant_db_->Insert(participant.name, participant);
    ROS_INFO("Done inserting");
  }
}

Task* Experiment::GetTask(Participant& participant, const string& task_name) {
  for (size_t i = 0; i < participant.tasks.size(); ++i) {
    const Task& saved_task = participant.tasks[i];
    if (saved_task.name == task_name) {
      return &participant.tasks[i];
    }
  }
  return NULL;
}

// Assumes both the cloud and the ROI are in base_link.
void Experiment::ComputeLandmark(const Roi3D& roi,
                                 const sensor_msgs::PointCloud2& scene,
                                 rapid_msgs::StaticCloud* static_cloud) {
  static_cloud->roi = roi;
  static_cloud->parent_frame_id = kBaseFrame;
  static_cloud->base_to_camera.rotation.w = 1;

  PointCloud<PointXYZRGB>::Ptr pcl_cloud(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(scene, *pcl_cloud);
  pcl::CropBox<PointXYZRGB> crop;
  crop.setInputCloud(pcl_cloud);
  Eigen::Vector4f min_pt;
  min_pt << roi.transform.translation.x - roi.dimensions.x / 2,
      roi.transform.translation.y - roi.dimensions.y / 2,
      roi.transform.translation.z - roi.dimensions.z / 2, 0;
  Eigen::Vector4f max_pt;
  max_pt << roi.transform.translation.x + roi.dimensions.x / 2,
      roi.transform.translation.y + roi.dimensions.y / 2,
      roi.transform.translation.z + roi.dimensions.z / 2, 0;
  crop.setMin(min_pt);
  crop.setMax(max_pt);

  PointCloud<PointXYZRGB> out;
  crop.filter(out);
  pcl::toROSMsg(out, static_cloud->cloud);
}

void Experiment::UpdateParams() {
  double sample_ratio;
  int max_samples;
  double max_sample_radius;
  int max_neighbors;
  double feature_threshold;
  int num_candidates;
  double fitness_threshold;
  double sigma_threshold;
  double nms_radius;
  int min_results;
  ros::param::param<double>("sample_ratio", sample_ratio, 0.01);
  ros::param::param<int>("max_samples", max_samples, 1000);
  ros::param::param<double>("max_sample_radius", max_sample_radius, 0.1);
  ros::param::param<int>("max_neighbors", max_neighbors, 400);
  ros::param::param<double>("feature_threshold", feature_threshold, 1500);
  ros::param::param<int>("num_candidates", num_candidates, 100);
  ros::param::param<double>("fitness_threshold", fitness_threshold, 0.0055);
  ros::param::param<double>("sigma_threshold", sigma_threshold, 2);
  ros::param::param<double>("nms_radius", nms_radius, 0.03);
  ros::param::param<int>("min_results", min_results, 0);
  ROS_INFO(
      "Parameters:\n"
      "sample_ratio: %f\n"
      "max_samples: %d\n"
      "max_sample_radius: %f\n"
      "max_neighbors: %d\n"
      "feature_threshold: %f\n"
      "num_candidates: %d\n"
      "fitness_threshold: %f\n"
      "sigma_threshold: %f\n"
      "nms_radius: %f\n"
      "min_results: %d\n",
      sample_ratio, max_samples, max_sample_radius, max_neighbors,
      feature_threshold, num_candidates, fitness_threshold, sigma_threshold,
      nms_radius, min_results);

  if (pose_estimator_.heat_mapper()->name() == "cnn") {
    ROS_ERROR("CNN heat mapper not enabled, update the code.");
    return;
  } else if (pose_estimator_.heat_mapper()->name() == "fpfh") {
    ROS_ERROR("FPFH heat mapper not enabled, update the code.");
    return;
  } else if (pose_estimator_.heat_mapper()->name() == "template_matching") {
    ROS_ERROR("Template matching heat mapper not enabled, update the code.");
    return;
  } else if (pose_estimator_.heat_mapper()->name() == "random") {
    rapid::perception::RandomHeatMapper* mapper =
        static_cast<rapid::perception::RandomHeatMapper*>(
            pose_estimator_.heat_mapper());
    mapper->set_sample_ratio(sample_ratio);
    mapper->set_max_samples(max_samples);
  }
  pose_estimator_.set_num_candidates(num_candidates);
  pose_estimator_.set_fitness_threshold(fitness_threshold);
  pose_estimator_.set_sigma_threshold(sigma_threshold);
  pose_estimator_.set_nms_radius(nms_radius);
  pose_estimator_.set_min_results(min_results);
}

void Experiment::ClearTestVisualization() {
  PointCloud<PointXYZRGB> pcl_blank;
  PointXYZRGB blank_pt;
  blank_pt.a = 0;
  pcl_blank.push_back(blank_pt);
  sensor_msgs::PointCloud2 blank;
  pcl::toROSMsg(pcl_blank, blank);
  blank.header.frame_id = kBaseFrame;
  alignment_pub_.publish(blank);
  output_pub_.publish(blank);
}

void Experiment::ClearAlignmentVisualization() {
  PointCloud<PointXYZRGB> pcl_blank;
  PointXYZRGB blank_pt;
  blank_pt.a = 0;
  pcl_blank.push_back(blank_pt);
  sensor_msgs::PointCloud2 blank;
  pcl::toROSMsg(pcl_blank, blank);
  blank.header.frame_id = kBaseFrame;
  alignment_pub_.publish(blank);
}

void Experiment::TaskOrder(const string& participant_name, const int num_tasks,
                           std::vector<int>* order) {
  order->clear();
  for (int i = 0; i < num_tasks; ++i) {
    order->push_back(i);
  }
  int hash = 0;
  for (size_t i = 0; i < participant_name.size(); ++i) {
    hash += participant_name.at(i);
    participant_name[i];
  }
  std::srand(hash);
  std::random_shuffle(order->begin(), order->end());
}

string Experiment::TaskName(const string& participant_name,
                            const int task_number) {
  std::vector<int> task_order;
  TaskOrder(participant_name, task_list_.size(), &task_order);
  if (static_cast<size_t>(task_number) >= task_order.size()) {
    return kEndTask;
  }
  int task_name = task_order[task_number];
  return task_list_[task_name];
}

string Experiment::TrainSceneName(const string& task_name) { return task_name; }
string Experiment::TestSceneName(const string& task_name) {
  return task_name + "_test";
}
}  // namespace study
