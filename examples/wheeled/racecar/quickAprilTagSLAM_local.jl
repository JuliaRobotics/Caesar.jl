# Local compute version

using Caesar
using YAML

function loadConfig()
  cfg = Dict{Symbol,Any}()
  data =   YAML.load(open(joinpath(Pkg.dir("Caesar"),"examples","wheeled","racecar","cam_cal.yml")))
  bRc = eval(parse("["*data["extrinsics"]["bRc"][1]*"]"))
  cfg[:bRc] = bRc
  cfg
end


cfg = loadConfig()

datafolder = ENV["HOME"]*"/data/racecar/smallloop/"
@load datafolder*"tag_det_per_pose.jld" tag_bag





#
