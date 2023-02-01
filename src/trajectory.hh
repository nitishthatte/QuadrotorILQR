
#pragma once

#include <ostream>
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

template <class ModelT>
std::ostream &operator<<(std::ostream &out, const TrajectoryPoint<ModelT> &pt) {
  out << "{\n"
      << "\ttime_s: " << pt.time_s << ",\n"
      << "\tstate: " << pt.state << ",\n"
      << "\tcontrol:" << pt.control.transpose() << "\n"
      << "}";
  return out;
}

template <class ModelT>
std::ostream &operator<<(std::ostream &out, const Trajectory<ModelT> &traj) {
  out << "{\n";
  for (const auto &pt : traj) {
    out << pt << ",\n";
  }
  out << "}";

  return out;
}

}  // namespace src