
#pragma once

#include <vector>

#include "src/dynamics.hh"

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
std::ostream &operator<<(std::ostream &out, const TrajectoryPoint<ModelT> &pt) {
  return out << "time_s: " << pt.time_s << ", state: " << pt.state
             << ", control: " << pt.control;
}

template <class ModelT>
using Trajectory = std::vector<TrajectoryPoint<ModelT>>;

}  // namespace src