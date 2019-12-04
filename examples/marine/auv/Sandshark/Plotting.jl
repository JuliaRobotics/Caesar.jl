
using Gadfly, Fontconfig, Cairo
using RoMEPlotting
Gadfly.set_default_plot_size(35cm,25cm)


function layerBeamPatternRose(bear::BallTreeDensity;
                              scale::Float64=1.0,
                              c=colorant"magenta",
                              wRr=TU.R(0.0),
                              wTRr=zeros(2) )
  #
  tp = reshape([0:0.01:(2pi);], 1, :)
  belp = scale*bear(tp)
  Gadfly.plot(x=tp, y=belp, Geom.path)
  belRose = zeros(2, length(tp))
  belRose[1,:] = belp

  idx = 0
  for rRc in TU.R.(tp)
    idx += 1
    belRose[:,idx] = wRr*rRc*belRose[:,idx] + wTRr
  end

  Gadfly.layer(x=belRose[1,:], y=belRose[2,:], Geom.path, Theme(default_color=c))
end



function plotSandsharkFromDFG(dfg::G; radix::Float64=1.8, scale::Float64=0.4) where G <: AbstractDFG
  # bearing range factors
  brf = ls(dfg, Pose2Point2BearingRange)
  #variables and poses connected to br factors
  brvars = 0 < length(brf) ? union(map(x->ls(dfg, x), brf)...) : Symbol[]
  brposes = setdiff(brvars, ls(dfg, r"l"))
  brposes .= sortVarNested(brposes)

  # get body frame beam estimate
  brfacts = Dict{Symbol, BallTreeDensity}()

  PL =  drawPosesLandms(dfg, spscale=2.5, contour=false, meanmax=:mean)

  # TODO: not matched properly
  for i in 1:length(brposes)
    @show brfi = intersect(ls(dfg, brposes[i]), brf)[1]
    fct = getFactorType(getFactor(dfg, brfi))
    brfacts[brposes[i]] = fct.bearing
    rVo = getKDEMax(getKDE(dfg, brposes[i]))
    pll = AMP.plotKDECircular([fct.bearing;], rVo=rVo, radix=radix, scale=scale)
    for lyr in pll.layers
      push!(PL.layers, lyr)
    end
  end

  PL
end


function drawPosesLandmarksAndOdo(fg::G,
                                  ppbrDict::Dict,
                                  navkeys::Array,
                                  X::Array,
                                  Y::Array,
                                  lblkeys::Array,
                                  lblX::Array,
                                  lblY::Array,
                                  gTitle::String="",
                                  drawhist=true  ) where G <: AbstractDFG
    #
    PLL = []
    # xx = sortVarNested(ls(fg, r"x"))
    # idx = 0
    # for idx in 1:length(xx)
    #   ep = epochs[idx]
    #   theta = getKDEMax(getVertKDE(fg, xx[idx]))
    #   wRr = TU.R(theta[3])
    #   pll = layerBeamPatternRose(ppbrDict[ep].bearing, wTRr=theta[1:2], scale=5.0 , wRr=wRr)
    #   push!(PLL, pll)
    # end
    #
    # pllandmarks = drawPosesLandms(fg, spscale=2.5, drawhist=drawhist)
    pllandmarks = plotSandsharkFromDFG(fg, scale=1.8)

    # Add odo
    navdf = DataFrame(
      ts = navkeys,
      x = X,
      y = Y
    )
    # pl = Gadfly.layer(navdf, x=:x, y=:y, Geom.path())
    push!(pllandmarks.layers, Gadfly.layer(navdf, x=:x, y=:y, Geom.path(), Theme(default_color=colorant"gray80"))[1])
    lbldf = DataFrame(
      ts = lblkeys .- lblkeys[1],
      x = lblX,
      y = lblY
    )
    push!(pllandmarks.layers, Gadfly.layer(lbldf, x=:x, y=:y, Geom.path(), Theme(default_color=colorant"magenta"))[1])
    # Gadfly.plot(pl.layers)

    # Make X/Y range same so no distorted
    # push!(PLL, Coord.Cartesian(xmin=-160.0,xmax=10.0,ymin=-135.0,ymax=35.0))
    pla = Gadfly.plot([PLL...;pllandmarks.layers; ]...,
        Guide.manual_color_key("Legend", ["Dead reckoning", "LBL Path", "Non-Gaussian SLAM"], ["gray80", "green", "light blue"]),
        Guide.title(gTitle))
    return pla
end



function plotRawBeamFormer(fg1::AbstractDFG, ppbrDict, NAV, interp_x, interp_y, interp_yaw; show::Bool=true, all::Bool=true)

  meta = readdlm(joinpath(getSolverParams(fg1).logpath, "metadata.txt"),',')
  mkpath(joinpath(getSolverParams(fg1).logpath, "rawplots"))


  files = String[]
  @showprogress "draw raw" for i in 1:(all ? size(meta,1) : 1)
    ep = Int(meta[i,2]) # 1531152901000000000
    bfspls = readdlm("/home/dehann/data/sandshark/full_wombat_2018_07_09/extracted/beamformer/particles/$(ep).csv",',')

    azipts = bfspls[:,1]
    aziptsw = TU.wrapRad.(azipts)
    aziprobl = kde!(azipts)
    npts = rand(aziprobl, 200)
    aziprob = manikde!(npts, Sphere1)

    plraw = Gadfly.plot(x=azipts, Geom.histogram(density=true), Guide.title("$ep, $(meta[i,3])"))
    pc = plotKDECircular(aziprob, title="recalc from raw (b-frame)")
    pbr= plotKDECircular(ppbrDict[ep].bearing, title="Pose2Point2BearingRange for fg (body)")
    pll = Gadfly.plot(x=[17;interp_x(ep)],y=[1.8;interp_y(ep)], Geom.line, Coord.Cartesian(xmin=-2.0+interp_x(ep),xmax=2.0+interp_x(ep),ymin=-  2.0+interp_y(ep),ymax=2.0+interp_y(ep)), Guide.title("n-frame, NAV yaw=$(round(interp_yaw(ep), digits=3))"))
    pbR= plotKDECircular(ppbrDict[ep].bearing, rVo=[interp_x(ep);interp_y(ep);interp_yaw(ep)], title="n-frame, NAV yaw=$(round(interp_yaw(ep), digits=3))")

    union!(pll.layers, pbR.layers)

    RoMEPlotting.addXYLineLayers!(pll, [interp_x(ep);], [interp_y(ep);], [interp_yaw(ep);], l=0.5)


    file = joinpath(getSolverParams(fg1).logpath, "rawplots", "bf_$(meta[i,3])_$ep.pdf")
    hstack(plraw, vstack(pc, pbr), pll) |> PDF(file)
    push!(files, file)
  end
  # compile into one PDF
  outfile = joinpath(getSolverParams(fg1).logpath, "bf_out.pdf")
  push!(files, outfile)
  run(`pdfunite $files`)

  !show ? nothing : (@async run(`evince $outfile`))
  return outfile
end



#
