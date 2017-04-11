# analyze depth of turtlebot sessions

using Caesar
using CloudGraphs
using Images, ImageMagick
using JSON
using Colors
using Gadfly
using IncrementalInference, KernelDensityEstimate
using Distributions


addrdict = nothing
include(joinpath(dirname(@__FILE__),"..","..","database","blandauthremote.jl"))
# prepare the factor graph with just one node
# (will prompt on stdin for db credentials)
cloudGraph, addrdict = standardcloudgraphsetup(addrdict=addrdict)

session = "SESSTURT21"
frd = fetchrobotdatafirstpose(cloudGraph, session)
camf = frd["CAMK"][1,1]
camc = frd["CAMK"][1,3]
tagwidth = 0.172

frd

dcamjl = DepthCamera(frd["CAMK"])
buildmesh!(dcamjl)



function imdrawline!(img, col; color=RGB(1.0,0.0,0.0) )
  for i in 1:size(img, 1)
    img[i, col] = color
  end
end


DISTS = Dict{Symbol, Vector{Float64}}()



# get subgraph around all landmarks
ids = getExVertexNeoIDs(cloudGraph.neo4j.connection, label="LANDMARK", session=session)

sfg = initfg(cloudgraph=cloudGraph, sessionname=session)
fetchsubgraph!(sfg, Int[ids[i][2] for i in 1:length(ids)], numneighbors=2 )

# writeGraphPdf(sfg)
# run(`evince fg.pdf`)

xx,ll = ls(sfg)

# cloudimshow(sfg.cg, session, :x2)

i = 0
# first for loop
for lsym in ll
  fcts = ls(sfg, lsym)
  # psoi = ls2(sfg,lsym)

  # second for loop
  for fct in fcts

    ve = nothing
    verts = lsf(sfg, fct)
    if 1 < length(verts)
      ve = getVert(sfg, fct, nt=:fnc)
      DISTS[fct] = Float64[getfnctype(ve).range.μ]
      # getfnctype(ve).bearing.μ

      pose = string(verts[1])[1] == 'x' ? verts[1] : verts[2]
      cv = getCloudVert(sfg.cg, sfg.sessionname, pose, bigdata=true)
      imdata = Caesar.getBigDataElement(cv, "keyframe_rgb").data;
      img = ImageMagick.readblob(imdata);
      depthdata = Caesar.getBigDataElement(cv, "depthframe_image").data;
      arr = bin2arr(depthdata, dtype=Float32) # should also store dtype for arr in Mongo
      depthimg = Array{Float64,2}(reshape(arr, frd["imshape"][2], frd["imshape"][1])')
      depthcloud = reconstruct(dcamjl, depthimg)

      measimgcol = round(Int,camc-camf*tan(getfnctype(ve).bearing.μ)) # measurement center
      deptdist = Base.mean(depthcloud[260:270,measimgcol,3])
      push!(DISTS[fct], deptdist)
      imdrawline!(img, measimgcol)
      roiimg = roi(img,240,measimgcol)
      @async imshowhack(roiimg)
      i+=1
      filename = joinpath("results","turtroi$(i).png")
      ImageMagick.save_(filename, roiimg)

      slamdist = getRangeKDEMax2D(sfg.cg, session, pose, lsym)
      push!(DISTS[fct], slamdist)
    end
  end
end


run(`mkdir -p results`)




cv = getCloudVert(cloudGraph, session, xx[1], bigdata=true);
sleep(0.5)
imdata = Caesar.getBigDataElement(cv, "keyframe_rgb").data;
img = ImageMagick.readblob(imdata);

roi(img,0,0)


# DISTS21 = deepcopy(DISTS)
# DISTS38 = deepcopy(DISTS)
# DISTS45 = deepcopy(DISTS)


cat21 = hcat(values(DISTS21)...)'
cat38 = hcat(values(DISTS38)...)'
cat45 = hcat(values(DISTS45)...)'

MSDISTS = vcat(cat21,cat38,cat45)





# Generate figure

vals = MSDISTS[:,1]-MSDISTS[:,2]
use2 = abs(vals) .< 1000
Nor2 = fit(Normal, vals[use2])

vals = MSDISTS[:,1]-MSDISTS[:,3]
use3 = abs(vals) .< 10
Nor3 = fit(Normal, vals[use3])

theme1 = Theme(key_max_columns=3, key_position=:top, guide_title_position=:left,
          key_label_font_size=10pt, background_color=colorant"white")


ps = 15

pl = plot(
layer(x -> pdf(Normal(0.0,0.4),x)*ps, -1.5,1.5, Theme(default_color=colorant"red",line_width=4pt)),
layer(x -> pdf(Nor2,x)*ps, -1.5,1.5, Theme(default_color=colorant"orange",line_width=2pt)),
layer(x -> pdf(Nor3,x)*ps, -1.5,1.5, Theme(default_color=colorant"green",line_width=2pt)),
layer(x=(MSDISTS[use2,1]-MSDISTS[use2,2]),Geom.histogram(bincount=18), Theme(default_color=color("orange"))),
layer(x=(MSDISTS[use3,1]-MSDISTS[use3,3]),Geom.histogram(bincount=18), Theme(default_color=color("green"))),
Guide.manual_color_key("",["Structured light, μ=$(round(Nor2.μ,4)), σ=$(round(Nor2.σ,3)), (not used yet)";
                           "SLAM fit, μ=$(round(Nor3.μ,4)), σ=$(round(Nor3.σ,3)), (monocular AprilTags)";
                           "Measurement model, μ=0, σ=0.4"],
                           ["orange","green","red"]),
Guide.xlabel("Post inference range distributions to AprilTags"),
Guide.yticks(ticks=nothing),
theme1
)

Gadfly.draw(PDF("turtrangedistr.pdf",14cm, 10cm),pl)
Gadfly.draw(PNG("turtrangedistr.png",14cm, 10cm),pl)
@async run(`evince turtrangedistr.pdf`)

#
