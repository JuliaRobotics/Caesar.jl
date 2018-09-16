function layerBeamPatternRose(bear::BallTreeDensity; scale::Float64=1.0, c=colorant"magenta", wRr=TU.R(0.0),wTRr=zeros(2))
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

function drawPosesLandmarksAndOdo(fg, ppbrDict, navkeys, X, Y, lblX, lblY, gTitle)
    PLL = []
    xx, = ls(fg)
    idx = 0
    for ep in epochs[1:end]
      idx += 1
      theta = getKDEMax(getVertKDE(fg, xx[idx]))
      wRr = TU.R(theta[3])
      pll = layerBeamPatternRose(ppbrDict[ep].bearing, wRr=wRr, wTRr=theta[1:2], scale=5.0)
      push!(PLL, pll)
    end

    pllandmarks = drawPosesLandms(fg, spscale=2.5, drawhist=false)
    # Add odo
    navdf = DataFrame(
      ts = navkeys,
      x = X,
      y = Y
    )
    # pl = Gadfly.layer(navdf, x=:x, y=:y, Geom.path())
    push!(pllandmarks.layers, Gadfly.layer(navdf, x=:x, y=:y, Geom.path(), Theme(default_color=colorant"red"))[1])
    lbldf = DataFrame(
      ts = lblkeys - lblkeys[1],
      x = lblX,
      y = lblY
    )
    push!(pllandmarks.layers, Gadfly.layer(lbldf, x=:x, y=:y, Geom.path(), Theme(default_color=colorant"green"))[1])
    # Gadfly.plot(pl.layers)

    # Make X/Y range same so no distorted
    # push!(PLL, Coord.Cartesian(xmin=-160.0,xmax=10.0,ymin=-135.0,ymax=35.0))
    pla = plot([PLL;pllandmarks.layers; ]...,
        Guide.manual_color_key("Legend", ["Particle Filter Est.", "LBL Path", "Non-Gaussian SLAM"], ["red", "green", "light blue"]),
        Guide.title(gTitle))
    return pla
end
