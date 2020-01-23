# common utils for applications -- staging area towards RoME itself

using RoME

@enum HandlerStateMachine HSMReady HSMHandling HSMOverlapHandling HSMBlocking
@enum SolverStateMachine SSMReady SSMConsumingSolvables SSMSolving

# Accumulate delta X values using FGOS
# TODO, refactor into RoME
function devAccumulateOdoPose2(DX::Array{Float64,2},
                               X0::Vector{Float64}=zeros(3);
                               P0=1e-3*Matrix(LinearAlgebra.I, 3,3),
                               Qc=1e-6*Matrix(LinearAlgebra.I, 3,3),
                               dt::Float64=1.0  )
  #
  # entries are rows with columns dx,dy,dtheta
  @assert size(DX,2) == 3
  mpp = MutablePose2Pose2Gaussian(MvNormal(X0, P0) )
  nXYT = zeros(size(DX,1), 3)
  for i in 1:size(DX,1)
    RoME.accumulateDiscreteLocalFrame!(mpp,DX[i, :],Qc,dt)
    nXYT[i,:] .= mpp.Zij.μ
  end

  return nXYT
end
