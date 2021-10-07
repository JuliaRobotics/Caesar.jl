module CommonUtils

using Caesar
using Images
using FileIO
using Cairo
using RoMEPlotting

export plotSLAM2D_KeyAndSim, plotHMSLevel


@deprecate buildDEMSimulated(w...;kw...) RoME.generateField_CanyonDEM(w...;kw...)
@deprecate getSampleDEM(w...;kw...) RoME.generateField_CanyonDEM(w...;kw...)
@deprecate loadDEM!(w...;kw...) RoME._buildGraphScalarField(w...;kw...)

@deprecate plotSLAM2D_KeyAndSim(w...;kw...) RoMEPlotting.plotSLAM2D_KeyAndRef(w...;kw...)



## plot some of the ROIs

function plotHMSLevel(fg::AbstractDFG, 
                      lbl::Symbol; 
                      coord=Coord.cartesian(xmin=-9000, xmax=9000, ymin=-9000,ymax=9000)  )
  #
  loc = getPPE(fg, lbl, :simulated).suggested
  plp = plot( x=[loc[1]], y=[loc[2];], 
              Geom.point, 
              Guide.title(string(lbl)), 
              Theme(default_color=colorant"purple") )
  #
  fct = intersect(ls(fg, lbl), lsf(fg, tags=[:DEM;]))[1]
  plk = getFactorType(fg[fct]).Z.densityFnc |> plotKDE;
  
  #
  union!(plp.layers, plk.layers); 
  plp.coord = coord
  plp
end


end