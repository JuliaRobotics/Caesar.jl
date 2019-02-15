
symbolmatch(s::Symbol, st::AS) where {AS <: AbstractString} = ismatch(Regex("$(st)"), string(s))
symbolmatch(s::Symbol, st::Char) = symbolmatch(s, string(st))

landmarksymbol(i::Int) = Symbol("l$i")
landmarknumber(sy::Symbol) = parse(Int, string(sy)[2:end])

function addoccurance!(lmo::Dict{Symbol, Int}, lm::Symbol)
  lmo[lm] = haskey(lmo, lm) ? lmo[lm]+1 : 1
  nothing
end

"""
    $(SIGNATURES)

For multiple measurements::Dict{Int,T} with unique Int signature, add "LANDMARK" `;softtype=Point2` to the fg::FactorGraph.  Type T not used in this function.
"""
function findAddNewLandmarks!(fgl::FactorGraph, lmoccur::Dict{Symbol, Int}, measurements::Dict{Int, T}; min_occurance::Int=2, softtype=Point2) where T
  # Count occurances of each landmark
  lmsyms = landmarksymbol.(collect(keys(measurements)))
  addoccurance!.(lmoccur, lmsyms)

  # add new landmarks not already in factor graph
  lmsinfgl = ls(fgl)[2]
  for lm in lmsyms
    if lmoccur[lm] >= min_occurance
      if !(lm in lmsinfgl)
        # add a new landmark
        println("adding lm=$(lm)")
        addVariable!(fgl, lm, softtype, labels=["LANDMARK";])
      end
      # @show measurements[landmarknumber(lm)]
    end
  end
  nothing
end

function findAddLandmFactorsByPose!(fgl::FactorGraph, psym::Symbol, meas::Dict{Int, T};
                    range_sigma::Float64=1.0, bearing_sigma::Float64=0.05, N::Int=100) where T
  lmsyms = landmarksymbol.(collect(keys(meas)))
  posels = ls(fgl, psym)
  lmsinfg = ls(fgl)[2]
  lms = intersect(lmsyms, lmsinfg)
  for lm in lms
    btwfcts = intersect(ls(fgl, lm), posels)
    if length(btwfcts) == 0
      z_rb = meas[landmarknumber(lm)]
      println("addFactor! $(psym) -- $(lm), z_rb=$(z_rb)")
      pp = Pose2Point2BearingRange( Normal(z_rb[2],bearing_sigma),Normal(z_rb[1],range_sigma) )
      addFactor!(fgl, [psym; lm], pp)
    end
  end
  nothing
end


function drive!(fgl::FactorGraph, idx::Int, odos::Dict, meas::Dict, lmoccurl::Dict, Podo;
                N::Int=100, min_occurance::Int=2 )
  prev, X, nextn = getLastPose2D(fgl)
  vp, fp = addOdoFG!(fgl, nextn, odos[idx][1:3], Podo, N=N)

  findAddNewLandmarks!(fgl, lmoccurl, meas[idx], min_occurance=min_occurance)

  # walk backwards
  donels = Symbol[]
  for i in 0:(min_occurance-1)
    @show psym = Symbol("x$(idx-i-1)") # off by one
    if idx-i > 0
      currmeas = meas[idx-i]
      findAddLandmFactorsByPose!(fgl, psym, currmeas, N=N)
    end
  end
  nothing
end

function donextframe!(fgl::FactorGraph, idx::Int, odos::Dict, meas::Dict, lmoccurl::Dict, Podo; N::Int=100)
  drive!(fgl, idx, odos, meas, lmoccurl, Podo, N=N)

  tree = wipeBuildNewTree!(fgl, drawpdf=true)
  inferOverTree!(fgl, tree,N=N)

  drawPosesLandms(fgl)
end
