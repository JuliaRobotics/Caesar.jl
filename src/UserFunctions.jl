# user functions

function identitypose6fg(;
      N::Int=100,
      initCov::Array{Float64,2}=0.001*eye(6) )
  #
  fg = initfg()
  N = 100

  println("Adding PriorPose3 and :x1 to graph...")
  pts = 0.01*randn(6,N)

  v0 = addNode!(fg, :x1,  pts,  N=N)
  initPosePrior = PriorPose3(SE3(0), initCov)
  f1  = addFactor!(fg,[v0], initPosePrior)
  return fg
end


function solveandvisualize(fg::FactorGraph,
  vc::VisualizationContainer;
  refresh::Number=1.0,
  densitymeshes::Vector{Symbol}=Symbol[])
  # draw while solving[1]==true
  solving = [true]
  @async begin
    while solving[1]
      println(".")
      visualizeallposes!(vc, fg)
      i = 1
      for dm in densitymeshes
        i+=1
        visualizeDensityMesh!(vc, fg, dm, meshid=i)
      end
      sleep(1)
    end
  end

  # solve
  tree = wipeBuildNewTree!(fg)
  @time inferOverTree!(fg, tree)
  solving[1]=false;

end
