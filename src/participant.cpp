#include "landmarks_study/participant.h"

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "landmarks_study/experiment_constants.h"

namespace study {
std::vector<int> TaskOrder(const std::string& participant_id) {
  std::vector<int> order(kNumTasks);
  for (int i = 0; i < kNumTasks; ++i) {
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

std::string TaskId(const std::string& participant_id, const int task_number) {
  std::vector<int> task_order = TaskOrder(participant_id);
  int task_id = task_order[task_number];
  if (task_id == 0) {
    return "cans";
  } else {
    return "";
  }
}
}  // namespace study
