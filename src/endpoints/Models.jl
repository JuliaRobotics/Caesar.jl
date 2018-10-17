# http://juliastats.github.io/StatsBase.jl/stable/weights.html
mutable struct Packed_AliasingScalarSampler
  samples::Vector{Float64}
  weights::Vector{Float64}
end

## avert your eyes, C++ below
# class MvNormal : public Distribution {
#   std::vector<double> mean_;
#   std::vector<std::vector<double>> cov_;
#
# public:
#   MvNormal(const std::vector<double> &mean,
#            const std::vector<std::vector<double>> &cov)
#       : mean_(mean), cov_(cov) {}
#   json ToJson(void) const {
#     json j;
#     j["mean"] = mean_;
#     j["cov"] = cov_;
#     return (j);
#   }
# };
