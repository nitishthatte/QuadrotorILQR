
#pragma once

#include "src/ilqr_options.hh"
#include "src/ilqr_options.pb.h"

namespace src {

proto::LineSearchParams to_proto(const LineSearchParams &params);
LineSearchParams from_proto(const proto::LineSearchParams &proto);

proto::ConvergenceCriteria to_proto(const ConvergenceCriteria &crit);
ConvergenceCriteria from_proto(const proto::ConvergenceCriteria &proto);

proto::ILQROptions to_proto(const ILQROptions &opts);
ILQROptions from_proto(const proto::ILQROptions &proto);

}  // namespace src