#ifndef _STUDY_PARTICIPANT_H_
#define _STUDY_PARTICIPANT_H_

#include <string>
#include <vector>

namespace study {
std::vector<int> TaskOrder(const std::string& participant_id);
std::string TaskId(const std::string& participant_id, const int task_number);
}  // namespace study

#endif  // _STUDY_PARTICIPANT_H_
