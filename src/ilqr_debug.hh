#pragma once

#include <vector>

#include "src/trajectory.hh"

namespace src {

template <class ModelT>
struct ILQRIterDebug {
  Trajectory<ModelT> trajectory;
  double cost;
};

template <class ModelT>
bool operator==(const ILQRIterDebug<ModelT> &lhs,
                const ILQRIterDebug<ModelT> &rhs) {
  return lhs.trajectory == rhs.trajectory && lhs.cost == rhs.cost;
}

template <class ModelT>
using ILQRDebug = std::vector<ILQRIterDebug<ModelT>>;

}  // namespace src