#ifndef _STUDY_PARTICIPANT_H_
#define _STUDY_PARTICIPANT_H_

#include <string>
#include <vector>

namespace study {
std::vector<int> TaskOrder(const std::string& participant_id,
                           const int num_tasks);
std::string TaskId(const std::string& participant_id, const int task_number,
                   const std::vector<std::string>& task_list);
}  // namespace study

#endif  // _STUDY_PARTICIPANT_H_
