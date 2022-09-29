
#include "src/ilqr_options_to_proto.hh"

#include <algorithm>

namespace src {

proto::LineSearchParams to_proto(const LineSearchParams &params) {
  proto::LineSearchParams proto{};
  proto.set_step_update(params.step_update);
  proto.set_desired_reduction_frac(params.desired_reduction_frac);
  proto.set_max_iters(params.max_iters);
  return proto;
}

LineSearchParams from_proto(const proto::LineSearchParams &proto) {
  return LineSearchParams{
      .step_update = proto.step_update(),
      .desired_reduction_frac = proto.desired_reduction_frac(),
      .max_iters = proto.max_iters()};
}

proto::ConvergenceCriteria to_proto(const ConvergenceCriteria &crit) {
  proto::ConvergenceCriteria proto{};
  proto.set_rtol(crit.rtol);
  proto.set_atol(crit.atol);
  proto.set_max_iters(crit.max_iters);
  return proto;
}

ConvergenceCriteria from_proto(const proto::ConvergenceCriteria &proto) {
  return ConvergenceCriteria{proto.rtol(), proto.atol(), proto.max_iters()};
}

proto::ILQROptions to_proto(const ILQROptions &opts) {
  proto::ILQROptions proto;
  *proto.mutable_line_search_params() = to_proto(opts.line_search_params);
  *proto.mutable_convergence_criteria() = to_proto(opts.convergence_criteria);
  proto.set_populate_debug(opts.populate_debug);
  return proto;
}

ILQROptions from_proto(const proto::ILQROptions &proto) {
  return {.line_search_params = from_proto(proto.line_search_params()),
          .convergence_criteria = from_proto(proto.convergence_criteria()),
          .populate_debug = proto.populate_debug()};
}

}  // namespace src