
function plotRangeOnlyHandyCompare(fg,
                                   expID,
                                   igt,
                                   posData,
                                   dposData,
                                   rangewindow,
                                   datadir; savefile="/tmp/caesar/test.pdf" )
  #
  plk= [];
  push!(plk,plotBeaconGT(igt));
  # plotBeaconContours!(plk,fg);
  pl1 = plotSASDefault(fg,expID, posData, igt, dposData, datadir=datadir, savedir="/tmp/ignore.pdf");

  for var in rangewindow
      mysym = Symbol("x$var")
      push!(plk, plotPoint(getVal(fg,mysym), colorIn = colorant"orange"))
  end

  L1ac, = predictbelief(fg, :l1,ls(fg, :l1))
  push!(plk,layer(x=L1ac[1,:],y=L1ac[2,:],Geom.histogram2d(xbincount=400, ybincount=400)))

  plotKDEMeans!(plk,fg);
  push!(plk,plotPath(posData));
  push!(plk,plotPath(dposData,colorIn=colorant"blue"));
  if expID == "dock"
      push!(plk, Coord.cartesian(xmin=-40, xmax=140, ymin=-140, ymax=30,fixed=true))
  elseif expID == "drift"
      push!(plk, Coord.cartesian(xmin=20, xmax=200, ymin=-220, ymax=0,fixed=true))
  end
  # plot new contours too
  tmp = plotKDE(manikde!(Point2, L1ac),levels=5,c=["black"])
  push!(plk, tmp.layers[1])

  pl2 = Gadfly.plot(plk...)
  pp = hstack(pl1, pl2)
  pp |> PDF(savefile)
  @async run(`evince $savefile`)
  pp
end
