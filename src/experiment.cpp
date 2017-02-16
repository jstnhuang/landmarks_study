#include "landmarks_study/experiment.h"

#include <string>
#include <vector>

#include "Eigen/Dense"
#include "mongodb_store/message_store.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/time.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rapid_msgs/Roi3D.h"
#include "rapid_msgs/StaticCloud.h"
#include "rapid_perception/box3d_roi_server.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/random_heat_mapper.h"

#include "landmarks_study/participant.h"
#include "landmarks_study/Participant.h"
#include "landmarks_study/Task.h"
#include "landmarks_study/UserAction.h"
#include "landmarks_study/experiment_constants.h"

using mongodb_store::MessageStoreProxy;
using landmarks_study::Participant;
using landmarks_study::Task;
using landmarks_study::UserAction;
using pcl::PointCloud;
using pcl::PointXYZRGB;

namespace study {
Experiment::Experiment(const MessageStoreProxy& participant_db,
                       const MessageStoreProxy& landmark_db,
                       const MessageStoreProxy& scene_db,
                       const rapid::perception::Box3DRoiServer& roi,
                       const ros::Publisher& scene_pub,
                       const ros::Publisher& alignment_pub,
                       const ros::Publisher& output_pub,
                       const rapid::perception::PoseEstimator& pose_estimator,
                       const std::vector<std::string>& task_list)
    : participant_db_(participant_db),
      landmark_db_(landmark_db),
      scene_db_(scene_db),
      roi_(roi),
      scene_pub_(scene_pub),
      alignment_pub_(alignment_pub),
      output_pub_(output_pub),
      pose_estimator_(pose_estimator),
      task_list_(task_list),
      participant_(),
      task_(),
      scene_(new sensor_msgs::PointCloud2),
      test_scene_(new sensor_msgs::PointCloud2),
      landmark_() {
  roi_.set_base_frame(kBaseFrame);
}

void Experiment::ProcessAction(const landmarks_study::UserAction& action) {
  const std::string& task_id =
      TaskId(action.participant_id, action.task_number, task_list_);
  if (action.action == landmarks_study::UserAction::LOAD) {
    // Save current ROI
    if (task_.id != "") {
      SaveTask();
    }
    ClearState();
    Load(action.participant_id, task_id);
    task_.actions.push_back(action);
    Edit(action.participant_id, task_id);
  } else if (action.action == landmarks_study::UserAction::EDIT) {
    Edit(action.participant_id, task_id);
    task_.actions.push_back(action);
  } else if (action.action == landmarks_study::UserAction::TEST) {
    Test(action.participant_id, task_id);
    task_.actions.push_back(action);
  } else if (action.action == landmarks_study::UserAction::SAVE) {
    SaveTask();
    task_.actions.push_back(action);
  } else {
  }
}

void Experiment::ClearState() {
  Participant participant;
  participant_ = participant;
  Task task;
  task_ = task;
  scene_.reset(new sensor_msgs::PointCloud2);
  test_scene_.reset(new sensor_msgs::PointCloud2);
}

void Experiment::SaveTask() {
  // Get the static cloud
  rapid_msgs::StaticCloud static_cloud;
  landmark_.roi = roi_.roi();
  ComputeLandmark(landmark_.roi, *scene_, &static_cloud);

  // Save the static cloud to the DB.
  std::string id = task_.landmark_id;
  std::vector<rapid_msgs::StaticCloud::Ptr> results;
  if (id == "" || !landmark_db_.queryID(id, results)) {
    id = landmark_db_.insert(static_cloud);
  } else {
    landmark_db_.updateID(task_.landmark_id, static_cloud);
  }
  ROS_INFO("Saved landmark for participant: %s, task: %s",
           participant_.id.c_str(), task_.id.c_str());

  // Update the task and participant data.
  task_.landmark_id = id;
  participant_.tasks.push_back(task_);

  // Save the participant.
  std::vector<landmarks_study::Participant::Ptr> results2;
  if (participant_db_.queryNamed(participant_.id, results2)) {
    ROS_INFO(
        "Updating participant %s, task: %s, landmark_id: %s, # actions: %ld",
        participant_.id.c_str(), task_.id.c_str(), task_.landmark_id.c_str(),
        task_.actions.size());
    participant_db_.updateNamed(participant_.id, participant_, true);
  } else {
    ROS_INFO(
        "Inserting participant %s, task: %s, landmark_id: %s, # actions: %ld",
        participant_.id.c_str(), task_.id.c_str(), task_.landmark_id.c_str(),
        task_.actions.size());
    participant_db_.insertNamed(participant_.id, participant_);
  }
}

void Experiment::Load(const std::string& participant_id,
                      const std::string& task_id) {
  ClearTestVisualization();

  // Load participant if already in the DB.
  GetParticipant(participant_id, &participant_);
  participant_.id = participant_id;
  if (participant_.tasks.size() > 0) {
    ROS_INFO("Participant %s already in DB.", participant_id.c_str());
  } else {
    ROS_INFO("Participant %s not in DB.", participant_id.c_str());
  }
  ROS_INFO("%ld tasks recorded for participant %s", participant_.tasks.size(),
           participant_.id.c_str());

  // Load task if already in the DB.
  GetTask(participant_, task_id, &task_);
  task_.id = task_id;
  if (task_.landmark_id != "") {
    ROS_INFO("Task %s already in DB.", participant_id.c_str());
  } else {
    ROS_INFO("Task %s not in DB.", participant_id.c_str());
  }
  ROS_INFO("%ld actions recorded for task %s", task_.actions.size(),
           task_.id.c_str());

  // Load scene
  GetScene(TrainSceneName(task_id), scene_);

  // Load landmark
  GetLandmark(task_.landmark_id, &landmark_);

  ROS_INFO("Loaded participant %s, task %s", participant_.id.c_str(),
           task_.id.c_str());
}

void Experiment::Edit(const std::string& participant_id,
                      const std::string& task_id) {
  ClearTestVisualization();
  if (landmark_.roi.dimensions.x == 0 || landmark_.roi.dimensions.y == 0 ||
      landmark_.roi.dimensions.z == 0) {
    roi_.Start();
  } else {
    roi_.Start(landmark_.roi.transform.translation.x,
               landmark_.roi.transform.translation.y,
               landmark_.roi.transform.translation.z,
               landmark_.roi.dimensions.x, landmark_.roi.dimensions.y,
               landmark_.roi.dimensions.z);
  }

  if (!scene_) {
    ROS_ERROR("Scene is null");
  }

  // Publish scene
  scene_pub_.publish(scene_);

  ROS_INFO("Editing ROI for participant %s, task %s", participant_.id.c_str(),
           task_.id.c_str());
}

void Experiment::Test(const std::string& participant_id,
                      const std::string& task_id) {
  // Get the static cloud
  landmark_.roi = roi_.roi();
  ComputeLandmark(landmark_.roi, *scene_, &landmark_);

  // Stop publishing the box
  roi_.Stop();

  // Publish the test scene.
  GetScene(TestSceneName(task_id), test_scene_);
  scene_pub_.publish(test_scene_);

  ROS_INFO("Testing landmark for participant %s, task %s",
           participant_.id.c_str(), task_.id.c_str());
  pcl::StopWatch watch;
  watch.reset();

  // Run CustomLandmarks
  PointCloud<PointXYZRGB>::Ptr landmark_cloud(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(landmark_.cloud, *landmark_cloud);
  PointCloud<PointXYZRGB>::Ptr test_scene_pcl(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(*test_scene_, *test_scene_pcl);

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
  pose_estimator_.set_roi(landmark_.roi);
  pose_estimator_.set_scene(test_scene_sampled);
  UpdateParams();
  std::vector<rapid::perception::PoseEstimationMatch> matches;
  pose_estimator_.Find(&matches);

  ClearAlignmentVisualization();

  ROS_INFO("Testing took %f seconds for participant %s, task %s",
           watch.getTimeSeconds(), participant_.id.c_str(), task_.id.c_str());
}

bool Experiment::GetParticipant(const std::string& participant_id,
                                Participant* participant) {
  ROS_INFO("Looking up participant %s", participant_id.c_str());
  std::vector<Participant::Ptr> results;
  bool success = participant_db_.queryNamed(participant_id, results, true);
  if (!success || results.size() == 0) {
    return false;
  }
  *participant = *results[0];
  return true;
}

bool Experiment::GetTask(const landmarks_study::Participant& participant,
                         const std::string& task_id,
                         landmarks_study::Task* task) {
  for (size_t i = 0; i < participant.tasks.size(); ++i) {
    const Task& saved_task = participant.tasks[i];
    if (saved_task.id == task_id) {
      *task = saved_task;
      return true;
    }
  }
  return false;
}

bool Experiment::GetScene(const std::string& scene_id,
                          sensor_msgs::PointCloud2::Ptr cloud) {
  ROS_INFO("Getting scene %s", scene_id.c_str());
  std::vector<sensor_msgs::PointCloud2::Ptr> results;
  bool success = scene_db_.queryNamed(scene_id, results, true);
  if (!success || results.size() == 0) {
    return false;
  }
  if (!(results[0])) {
    ROS_ERROR("Could not load scene %s", scene_id.c_str());
    return false;
  }
  *cloud = *results[0];
  return true;
}

bool Experiment::GetLandmark(const std::string& landmark_id,
                             rapid_msgs::StaticCloud* landmark) {
  ROS_INFO("Getting landmark %s", landmark_id.c_str());
  if (landmark_id == "") {
    return false;
  }
  std::vector<rapid_msgs::StaticCloud::Ptr> results;
  bool success = landmark_db_.queryID(landmark_id, results);
  if (!success || results.size() == 0) {
    return false;
  }
  *landmark = *results[0];
  return true;
}

// Assumes both the cloud and the ROI are in base_link.
void Experiment::ComputeLandmark(const rapid_msgs::Roi3D& roi,
                                 const sensor_msgs::PointCloud2& scene,
                                 rapid_msgs::StaticCloud* static_cloud) {
  static_cloud->roi = roi;
  static_cloud->parent_frame_id = kBaseFrame;
  static_cloud->base_to_camera.rotation.w = 1;

  PointCloud<PointXYZRGB>::Ptr pcl_cloud(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(*scene_, *pcl_cloud);
  pcl::CropBox<PointXYZRGB> crop_;
  crop_.setInputCloud(pcl_cloud);
  Eigen::Vector4f min_pt;
  min_pt << roi.transform.translation.x - roi.dimensions.x / 2,
      roi.transform.translation.y - roi.dimensions.y / 2,
      roi.transform.translation.z - roi.dimensions.z / 2, 0;
  Eigen::Vector4f max_pt;
  max_pt << roi.transform.translation.x + roi.dimensions.x / 2,
      roi.transform.translation.y + roi.dimensions.y / 2,
      roi.transform.translation.z + roi.dimensions.z / 2, 0;
  crop_.setMin(min_pt);
  crop_.setMax(max_pt);

  PointCloud<PointXYZRGB> out;
  crop_.filter(out);
  pcl::toROSMsg(out, static_cloud->cloud);
}

std::string Experiment::TrainSceneName(const std::string& task_id) {
  return task_id;
}
std::string Experiment::TestSceneName(const std::string& task_id) {
  return task_id + "_test";
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
}  // namespace study
