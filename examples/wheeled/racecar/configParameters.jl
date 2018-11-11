include("parsecommands.jl")
using YAML

function loadConfig()
  cfg = Dict{Symbol,Any}()
  data =   YAML.load(open(joinpath(dirname(@__FILE__),"cam_cal.yml")))
  bRc = eval(Meta.parse("["*data["extrinsics"]["bRc"][1]*"]"))
  # convert to faster symbol lookup
  cfg[:extrinsics] = Dict{Symbol,Any}()
  cfg[:extrinsics][:bRc] = bRc
  cfg[:intrinsics] = Dict{Symbol,Any}()
  cfg[:intrinsics][:height] = data["left"]["intrinsics"]["height"]
  cfg[:intrinsics][:width] = data["left"]["intrinsics"]["width"]
  haskey(data["left"]["intrinsics"], "camera_matrix") ? (cfg[:intrinsics][:cam_matrix] = data["left"]["intrinsics"]["camera_matrix"]) : nothing
  cfg[:intrinsics][:cx] = cx#data["left"]["intrinsics"]["cx"]
  cfg[:intrinsics][:cy] = cy#data["left"]["intrinsics"]["cy"]
  cfg[:intrinsics][:fx] = fx#data["left"]["intrinsics"]["fx"]
  cfg[:intrinsics][:fy] = fy#data["left"]["intrinsics"]["fy"]
  cfg[:intrinsics][:k1] = data["left"]["intrinsics"]["k1"]
  cfg[:intrinsics][:k2] = data["left"]["intrinsics"]["k2"]
  cfg
end


cfg = loadConfig()

cw, ch = cfg[:intrinsics][:cx], cfg[:intrinsics][:cy]
fx = fy = 1.0*cfg[:intrinsics][:fx]
camK = [fx 0 cw; 0 fy ch; 0 0 1]
tagsize = 0.172
# k1,k2 = cfg[:intrinsics][:k1], cfg[:intrinsics][:k2] # makes it worse
k1, k2 = 0.0, 0.0

# tag extrinsic rotation
Rx = RotX(-pi/2)
Rz = RotZ(-pi/2)
bTc= LinearMap(Rz) ∘ LinearMap(Rx)


datadir = joinpath(ENV["HOME"],"data","racecar")

imgfolder = "images"
datafolder = ENV["HOME"]*"/data/racecar/$(folderName)/";

camidxs = 0:5:5
if folderName == "labrun2"
  camidxs =  0:5:1625
elseif folderName == "labrun3"
  @error "unknown image camidxs"
elseif folderName == "labrun4"
  @error "unknown image camidxs"
elseif folderName == "labrun5"
  camidxs =  0:5:1020
elseif folderName == "labrun6"
  camidxs =  0:5:1795
elseif folderName == "labrun7"
  camidxs =  0:5:2135
elseif folderName == "labrun8"
  @error "unknown image camidxs"
elseif folderName == "straightrun3"
  camidxs =  175:5:370
else
  @error "unknown Racecar data folder, $folderName"
end

# datafolder = ENV["HOME"]*"/data/racecar/straightrun3/"  # 175:5:370
# datafolder = ENV["HOME"]*"/data/racecar/labrun2/; camidxs =  0:5:1625
# datafolder = ENV["HOME"]*"/data/racecar/labrun3/"; # camidxs =
# datafolder = ENV["HOME"]*"/data/racecar/$(folderName)/"; camidxs =  0:5:1020
# datafolder = ENV["HOME"]*"/data/racecar/labrun6/"; camidxs =  0:5:1795
# datafolder = ENV["HOME"]*"/data/racecar/labrun7/"; camidxs =  0:5:2135
# datafolder = ENV["HOME"]*"/data/racecar/labrun8/"; camidxs =  0:5:
