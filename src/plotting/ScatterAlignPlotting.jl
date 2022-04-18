
@info "Caesar.jl is including ScatterAlign tools associated with Gadfly.jl."

using .Gadfly

export plotScatterAlign


"""
    $SIGNATURES

Plot name tuple output from [`overlayScatter`](@ref)

See also: [`overlayScatterMutate`](@ref)
"""
function plotScatterAlign(snt::NamedTuple; title::String="")
  N = size(snt.pP1,2)

  pP1 = Gadfly.layer(x=snt.pP1[1,:],y=snt.pP1[2,:],color=[1;zeros(N-1)])
  qP2 = Gadfly.layer(x=snt.qP2[1,:],y=snt.qP2[2,:],color=[0;ones(N-2);2])
  pP2_u = Gadfly.layer(x=snt.pP2_u[1,:],y=snt.pP2_u[2,:],color=[0;2*ones(N-1)])
  pP2_b = Gadfly.layer(x=snt.pP2_b[1,:],y=snt.pP2_b[2,:],color=[0;2*ones(N-1)])
  
  uo = haskey(snt, :user_offset) ? "$(round.(snt.user_offset, digits=3))" : ""

  ar = Gadfly.Coord.cartesian(; aspect_ratio=1)
  H1 = Gadfly.plot(pP1, qP2,      ar, Guide.title("body frame data."*title))
  H2 = Gadfly.plot(pP1, pP2_u,    ar, Guide.title("User transform: $(round.(snt.user_coords,digits=3))\nu_score=$(snt.u_score)"))
  H3 = Gadfly.plot(pP1, pP2_b,    ar, Guide.title("Best fit: $(snt.best_coords)\nb_score=$(snt.b_score)"))
  
  hstack(H1,H2, H3)
end

plotScatterAlign( sap::ScatterAlignPose2;
                  sample_count::Integer = sap.sample_count,
                  bw::Real = sap.bw,
                  kw... ) = plotScatterAlign(overlayScatterMutate(sap; sample_count, bw); kw...)
#                  


#