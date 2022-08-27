
#pragma once

#include <vector>

namespace src {

template <class ModelT>
struct TrajectoryPoint {
  double time_s;
  typename ModelT::State state;
  typename ModelT::Control control;
};

template <class ModelT>
bool operator==(const TrajectoryPoint<ModelT> &lhs,
                const TrajectoryPoint<ModelT> &rhs) {
  return lhs.time_s == rhs.time_s && lhs.control == rhs.control &&
         lhs.state == rhs.state;
}

template <class ModelT>
using Trajectory = std::vector<TrajectoryPoint<ModelT>>;

}  // namespace src