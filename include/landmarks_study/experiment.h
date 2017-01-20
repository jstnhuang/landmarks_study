#ifndef _STUDY_EXPERIMENT_H_
#define _STUDY_EXPERIMENT_H_

#include <string>

#include "mongodb_store/message_store.h"
#include "rapid_perception/box3d_roi_server.h"
#include "rapid_msgs/StaticCloud.h"
#include "rapid_perception/pose_estimation.h"

#include "landmarks_study/UserAction.h"
#include "landmarks_study/Participant.h"
#include "sensor_msgs/PointCloud2.h"

namespace study {
class Experiment {
 public:
  Experiment(const mongodb_store::MessageStoreProxy& participant_db,
             const mongodb_store::MessageStoreProxy& landmark_db,
             const mongodb_store::MessageStoreProxy& scene_db,
             const rapid::perception::Box3DRoiServer& roi,
             const ros::Publisher& scene_pub,
             const ros::Publisher& alignment_pub,
             const ros::Publisher& output_pub,
             const rapid::perception::PoseEstimator& pose_estimator);
  void ProcessAction(const landmarks_study::UserAction& action);

 private:
  // Save the most recent task.
  void ClearState();
  void SaveTask();
  void Load(const std::string& participant_id, const std::string& task_id);
  void Edit(const std::string& participant_id, const std::string& task_id);
  void Test(const std::string& participant_id, const std::string& task_id);

  bool GetParticipant(const std::string& participant_id,
                      landmarks_study::Participant* participant);
  bool SaveParticipant(const std::string& participant_id,
                       const landmarks_study::Participant& participant);
  bool GetTask(const landmarks_study::Participant& participant,
               const std::string& task_id, landmarks_study::Task* task);
  bool GetScene(const std::string& scene_id,
                sensor_msgs::PointCloud2::Ptr cloud);
  bool GetLandmark(const std::string& landmark_id,
                   rapid_msgs::StaticCloud* cloud);
  void ComputeLandmark(const rapid_msgs::Roi3D& roi,
                       const sensor_msgs::PointCloud2& scene,
                       rapid_msgs::StaticCloud* static_cloud);
  void UpdateParams();
  void ClearTestVisualization();
  void ClearAlignmentVisualization();

  std::string TrainSceneName(const std::string& task_id);
  std::string TestSceneName(const std::string& task_id);

  mongodb_store::MessageStoreProxy participant_db_;
  mongodb_store::MessageStoreProxy landmark_db_;
  mongodb_store::MessageStoreProxy scene_db_;
  rapid::perception::Box3DRoiServer roi_;
  ros::Publisher scene_pub_;
  ros::Publisher alignment_pub_;
  ros::Publisher output_pub_;
  rapid::perception::PoseEstimator pose_estimator_;

  // The experiment server assumes we are managing the state of the most
  // recently loaded participant/task.
  landmarks_study::Participant participant_;
  landmarks_study::Task task_;
  sensor_msgs::PointCloud2::Ptr scene_;
  sensor_msgs::PointCloud2::Ptr test_scene_;
  rapid_msgs::StaticCloud landmark_;
};
}  // namespace study

#endif  // _STUDY_EXPERIMENT_H_
