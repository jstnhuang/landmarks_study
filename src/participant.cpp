#include "landmarks_study/participant.h"

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

#include "landmarks_study/experiment_constants.h"

namespace study {
std::vector<int> TaskOrder(const std::string& participant_id,
                           const int num_tasks) {
  std::vector<int> order(num_tasks);
  for (int i = 0; i < num_tasks; ++i) {
    order[i] = i;
  }
  int hash = 0;
  for (size_t i = 0; i < participant_id.size(); ++i) {
    hash += participant_id.at(i);
    participant_id[i];
  }
  std::srand(hash);
  std::random_shuffle(order.begin(), order.end());
  return order;
}

std::string TaskId(const std::string& participant_id, const int task_number,
                   const std::vector<std::string>& task_list) {
  std::vector<int> task_order = TaskOrder(participant_id, task_list.size());
  if (static_cast<size_t>(task_number) >= task_order.size()) {
    return kEndTask;
  }
  int task_id = task_order[task_number];
  return task_list[task_id];
}
}  // namespace study
