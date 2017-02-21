#ifndef _STUDY_EXPERIMENT_H_
#define _STUDY_EXPERIMENT_H_

#include <map>
#include <string>
#include <vector>

#include "rapid_db/name_db.hpp"
#include "rapid_msgs/StaticCloud.h"
#include "rapid_perception/box3d_roi_server.h"
#include "rapid_perception/pose_estimation.h"
#include "sensor_msgs/PointCloud2.h"

#include "landmarks_study/Event.h"
#include "landmarks_study/Participant.h"
#include "landmarks_study/Task.h"

namespace study {
class Experiment {
 public:
  Experiment(rapid::db::NameDb* participant_db, rapid::db::NameDb* landmark_db,
             rapid::db::NameDb* scene_db,
             const rapid::perception::Box3DRoiServer& roi,
             const ros::Publisher& scene_pub,
             const ros::Publisher& alignment_pub,
             const ros::Publisher& output_pub,
             const ros::Publisher& output_markers_pub,
             const ros::Publisher& status_pub,
             const ros::Publisher& description_pub,
             const rapid::perception::PoseEstimator& pose_estimator,
             const std::vector<std::string>& task_list,
             const std::vector<std::string>& task_descriptions);

  // Main event handler.
  void ProcessEvent(const landmarks_study::Event& event);

 private:
  // Display a visualization of the demonstration scene.
  void Load(const landmarks_study::Event& event, const std::string& task_name,
            const std::string& task_description);

  // Start the ROI server.
  void Edit(const landmarks_study::Event& event, const std::string& task_name);

  // Load the test scene and evaluate the landmark.
  void Test(const landmarks_study::Event& event, const std::string& task_name);

  // Save the user's ROI.
  void Save(const landmarks_study::Event& event, const std::string& task_name);

  // Updates or inserts a participant into the database.
  bool SaveParticipant(const landmarks_study::Participant& participant);

  // Returns a pointer to the participant's instance of the given task.
  landmarks_study::Task* GetTask(landmarks_study::Participant& participant,
                                 const std::string& task_name);
  void ComputeLandmark(const rapid_msgs::Roi3D& roi,
                       const sensor_msgs::PointCloud2& scene,
                       rapid_msgs::StaticCloud* static_cloud);
  void UpdateParams();
  void ClearTestVisualization();
  void ClearAlignmentVisualization();

  // Return the order of the tasks.
  void TaskOrder(const std::string& participant, const int num_tasks,
                 std::vector<int>* order);
  // Return the name of the task for the (participant, task number) pair.
  void TaskInfo(const std::string& participant, const int task_number,
                std::string* task_name, std::string* task_description);

  // Return transformations of demonstration/test scene names.
  std::string TrainSceneName(const std::string& task_name);
  std::string TestSceneName(const std::string& task_name);

  bool GetScene(const std::string& scene_name,
                sensor_msgs::PointCloud2* output);

  rapid::db::NameDb* participant_db_;
  rapid::db::NameDb* landmark_db_;
  rapid::db::NameDb* scene_db_;
  rapid::perception::Box3DRoiServer roi_;
  ros::Publisher scene_pub_;
  ros::Publisher alignment_pub_;
  ros::Publisher output_pub_;
  ros::Publisher output_markers_pub_;
  ros::Publisher status_pub_;
  ros::Publisher description_pub_;
  rapid::perception::PoseEstimator pose_estimator_;
  std::vector<std::string> task_list_;
  std::vector<std::string> task_descriptions_;

  std::map<std::string, sensor_msgs::PointCloud2> scene_cache_;
};
}  // namespace study

#endif  // _STUDY_EXPERIMENT_H_
