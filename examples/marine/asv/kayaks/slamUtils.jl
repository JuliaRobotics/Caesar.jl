using DocStringExtensions: SIGNATURES

function importdata_nav(frames::Array{Int,1};
                        datadir::String="/media/data1/data/kayaks/20_gps_pos")

    # "/media/data1/data/kayaks/20_parsed" this data is windowed 10 elements referenced to the first
    # get position data
    #  logdir = wavedatadir*"/array_positions$(sasframe).csv"  # 85

    dataOut = zeros(length(frames),2);
    count = 0
    for i in frames
        count+=1
        navfile = datadir*"/nav$(i).csv"
        dataOut[count,:] = readdlm(navfile,',',Float64,'\n')
    end
    dataOut
end

function sanitycheck_nav(navdata)
    check = true
    errorout = 0;
    for i in 1:size(navdata,1)-1
        dist = abs.(navdata[i,:]-navdata[i+1,:]);
        if dist[1] > 3
            println("NAV_X is off by $(dist[1])")
            check = false
            errorout = i
        end
        if dist[2] > 3
            println("NAV_Y is off by $(dist[2])")
            check = false
            errorout = i
        end
    end
    check, errorout
end

"""
    $(SIGNATURES)

Load hydrophone waveform data from hard drive, after it has been processed from MOOS [a/b]log.  Processing from MOOS format can be done according to scripts/parseData.py -- Mei says so, call on @mc2922 if you can't find it...

Input:
- `frames` is an index into the time of the mission, i.e. an array of poses that you are calling (meaningless index unless 1Hz happens to be time).
- `element` describes which hydrophone to use on the physcal array (initial development had 5 elements and commonly using 2).
- `datadir` is the directory location where all the waveforms%d.csv exist.

Output:
- an array of waveforms where the first dimension is the length of the waveform in time (i.e. number of samples), and the second dimension is the number of frames you ask for.

Example:

```julia
frm = [1,2,3,4] # corresponds to files waveforms[1/2/3/4].csv
data = importdata_waveforms(frm, 2)
```

Future:
There might be a more memory 'in-place' implementation, which would change the API.
"""
function importdata_waveforms(frames::Array{Int,1},
                              element::Int;
                              datadir::String="/media/data1/data/kayaks/20_gps_pos")
    wavefile = datadir*"/waveform$(frames[1]).csv"
    tempData = readdlm(wavefile,',',Float64,'\n');
    dataOut = zeros(size(tempData,2),length(frames));
    count = 0
    for i in frames
        count+=1;
        wavefile = datadir*"/waveform$(i).csv"
        tempData = readdlm(wavefile,',',Float64,'\n');
        dataOut[:,count] = tempData[element,:];
    end
    dataOut
end

function cumulativeDrift!(cdata::AbstractArray,cstart::Array,σ::Array)
    cdrift = cstart
    for i in 1:size(cdata,1)
        cdata[i,:] += cdrift
        cdrift += σ.*randn(2)
    end
    nothing
end

function addsascluster_velpt!(fg::G,
                              poses::Vector{Symbol},
                              beacon::Symbol,
                              sasframes::Array{Int},
                              tstart;
                              rtkCov::Array{Float64,2}= diagm([0.1;0.1].^2),
                              element::Int=2,
                              autoinit::Bool=false,
                              datadir::String="/media/data1/data/kayaks/20_gps_pos",
                              N::Int=100  ) where G <: AbstractDFG

    if length(sasframes) != length(poses)
        error("Frames and Poses are different lengths")
    end

  numelems = length(poses)

  lss = union(ls(fg)...)

  posData = importdata_nav(sasframes,datadir=datadir);
  dposData = deepcopy(posData)
  cumulativeDrift!(dposData,[0.0;0],[0.05,0.05])
  waveformData = importdata_waveforms(sasframes,element,datadir=datadir);
  tcurrent = tstart*1_000_000

  for sym in poses
    addVariable!(fg, sym, DynPoint2(ut=tcurrent))
    tcurrent += 1_000_000
  end

  #Fix the first point
  #Interpolate velocity from GPS ground truth
  xdotp = posData[2,1] - posData[1,1];
  ydotp = posData[2,2] - posData[1,2];
  dpμ = [posData[1,1];posData[1,2];xdotp;ydotp];
  dpσ = diagm([0.1;0.1;0.1;0.1].^2)

  pp = DynPoint2VelocityPrior(MvNormal(dpμ,dpσ))
  addFactor!(fg, [poses[1];], pp, autoinit=true)
  IncrementalInference.doautoinit!(fg,[getVariable(fg,poses[1])])
  setValKDE!(getVariable(fg,poses[1]),kde!(getSample(getfnctype(getFactor(fg, Symbol("$(poses[1])f1"))),N)[1]))

  #Add odo factors
  for i in 2:numelems
      @show xdotp = dposData[i,1] - dposData[i-1,1]
      @show ydotp = dposData[i,2] - dposData[i-1,2]
      dpμ = [xdotp;ydotp;0;0];
      dpσ = diagm([0.5;0.5;0.1;0.1].^2)
      vp = VelPoint2VelPoint2(MvNormal(dpμ,dpσ))
      addFactor!(fg, [poses[i-1];poses[i]], vp, autoinit=true)
      IncrementalInference.doautoinit!(fg,[getVariable(fg,poses[i])])
      #factorsym = Symbol("$(poses[i-1])$(poses[i])f1")
      #setValKDE!(getVariable(fg,poses[i]),kde!(getSample(getfnctype(getVariable(fg, factorsym,nt=:fnc)),N)[1]))
      @show hackinit = ones(4,N).*[dposData[i-1,:];xdotp;ydotp] + rand(MvNormal(dpμ,dpσ),N)
      setVal!(getVariable(fg,poses[i]),hackinit)
  end

  sas2d = prepareSAS2DFactor(numelems, waveformData, rangemodel=:Correlator)
  addFactor!(fg, [beacon;poses], sas2d, autoinit=autoinit)

  dposData
end


#

function addsascluster_only_gps!(fg::G,
                                 poses::Vector{Symbol},
                                 beacon::Symbol,
                                 sasframes::Array{Int};
                                 rtkCov::Array{Float64,2}= diagm([0.1;0.1].^2),
                                 datadir::String="/media/data1/data/kayaks/20_gps_pos",
                                 element::Int=2,
                                 autoinit::Bool=false,
                                 N::Int=100  ) where G <: AbstractDFG

  # "/media/data1/data/kayaks/20_parsed" this data is windowed 10 elements referenced to the first
  # get position data
#  logdir = wavedatadir*"/array_positions$(sasframe).csv"  # 85

    if length(sasframes) != length(poses)
        error("Frames and Poses are different lengths")
    end

  numelems = length(poses)

  lss = union(ls(fg)...)

  posData = importdata_nav(sasframes, datadir=datadir);
  waveformData = importdata_waveforms(sasframes,element,datadir=datadir);
  count = 0

  for sym in poses
    count += 1
    addVariable!(fg, sym, Point2)
    pp = PriorPoint2(MvNormal(posData[count,:], rtkCov) )
    addFactor!(fg, [sym;], pp, autoinit=true)
    IncrementalInference.doautoinit!(fg,sym)
    setValKDE!(getVariable(fg,sym),kde!(getSample(getfnctype(getFactor(fg, Symbol("$(sym)f1"))),N)[1]))
  end

  sas2d = prepareSAS2DFactor(numelems, waveformData, rangemodel=:Correlator)
  addFactor!(fg, [beacon;poses], sas2d, autoinit=autoinit)

  nothing
end

function approxConvFwdBFRaw(fg,
                            poses,
                            beacon,
                            origin,
                            scale,
                            snrfloor;
                            N::Int=100  )
  #
  fctsym = Symbol(string(beacon,poses...,:f1))
  sas2d = IncrementalInference.getfnctype(getFactor(fg, fctsym))
  sas2d.debugging = true
  for ti in 1:Threads.nthreads() reset!(sas2d.threadreuse[ti].dbg) end
  predL1 = IncrementalInference.approxConv(fg, fctsym, beacon, N=N)

  allbeams = []
  for ti in 1:Threads.nthreads()
      for itr in eachindex(sas2d.threadreuse[ti].dbg.beams)
          push!(allbeams,sas2d.threadreuse[ti].dbg.beams[itr])
      end
  end
  @show size(allbeams[1])
  avg_beam = zeros(length(allbeams[1]))
  for i in 1:N
    avg_beam[:] = avg_beam + allbeams[i]
  end

  #avg_beam ./= sum(avg_beam)
  #avg_beam .-= quantile(avg_beam, snrfloor)
  #avg_beam[avg_beam .< 0.0] = 0.0

  circ_avg_beam = zeros(2,length(avg_beam))
  count = 0
  for th in range(0,2pi,length=length(avg_beam))
    count += 1
    circ_avg_beam[:,count] = R(th)*scale*[avg_beam[count];0.0]+origin
  end

  allbeams
end

function approxConvFwdBFlayer(fg,
                              poses,
                              beacon,
                              origin,
                              scale,
                              snrfloor::Float64=0.0;
                              N::Int=100  )
  #
  circ_avg_beam = approxConvFwdBFRaw(fg, poses, beacon, origin, scale, snrfloor, N=N)

  plPica = Gadfly.layer(x=circ_avg_beam[1,:], y=circ_avg_beam[2,:], Geom.path(),   Theme(default_color=colorant"magenta", line_width=2pt))

  plPica
end

"""
    $SIGNATURES

Plot side by side pair of poses and beacon for a particular SAS factor.

Notes:
- Specify non-default `filepath<:String`
- can show pdf via evince if `show::Bool`
"""
function plotSASPair(fg::G,
                     fsym::Symbol;
                     show::Bool=true,
                     filepath::AS="/tmp/test.pdf") where {G <: AbstractDFG, AS <: AbstractString}
    #
    PL = [];

    @show fsym
    @show vars = lsf(fg, fsym)
    beac = vars[1]

    for var in vars[2:end]
        X1 = getVal(getVariable(fg, var))
        push!(PL, layer(x=X1[1,:],y=X1[2,:], Geom.histogram2d))
    end

    # push!(PL, approxConvFwdBFlayer(fg, poses, :l1, posData[1,:], 2000 ))

    pl11 = Gadfly.plot(PL...);
    # pl.coord = Coord.Cartesian(xmin=10,xmax=30,ymin=-60,ymax=-45)

    pl21 = plotKDE(fg, vars[2:end], levels=1, dims=[1;2])

    L1v = getVariable(fg, beac)
    L1 = getVal(L1v)
    pl12 = Gadfly.plot(x=L1[1,:],y=L1[2,:], Geom.histogram2d)

    pl22 = plotKDE(getBelief(L1v), levels=3)

    # stack all images together
    plT = Gadfly.hstack(pl11,pl12)
    plB = Gadfly.hstack(pl21,pl22)
    plA = Gadfly.vstack(plT, plB)

    # if the user wants to see a file rendering
    if show
      plA |> PDF(filepath);  @async run(`evince $filepath`)
    end

    plA
end
