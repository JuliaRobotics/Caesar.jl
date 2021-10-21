
@info "Caesar.jl is including ScatterAlign tools associated with Gadfly.jl."

using .Gadfly

export plotScatterAlign

function plotScatterAlign(snt::NamedTuple)
  N = size(snt.pPp,2)

  plp = Gadfly.layer(x=snt.pPp[1,:],y=snt.pPp[2,:],color=[1;zeros(N-1)])
  plq = Gadfly.layer(x=snt.qPq[1,:],y=snt.qPq[2,:],color=[0;ones(N-2);2])
  plpq = Gadfly.layer(x=snt.pPq[1,:],y=snt.pPq[2,:],color=[0;ones(N-1)])
  plpq_best = Gadfly.layer(x=snt.pPq_best[1,:],y=snt.pPq_best[2,:],color=[0;ones(N-1)])
  
  H1 = Gadfly.plot(plp, plq, Guide.title("Raw samples"))
  H2 = Gadfly.plot(plp, plpq, Guide.title("User transform: $(snt.user_coords)"))
  H3 = Gadfly.plot(plp, plpq_best, Guide.title("Best fit: $(snt.best_coords)"))
  
  hstack(H1,H2, H3)
end

plotScatterAlign( sap::ScatterAlignPose2;
                  sample_count::Integer = sap.sample_count,
                  bw::Real = sap.bw ) = plotScatterAlign(overlayScatterMutate(sap; sample_count, bw))
                  


#