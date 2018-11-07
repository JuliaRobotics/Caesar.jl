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

# datafolder = ENV["HOME"]*"/data/racecar/straightrun3/"  # 175:5:370
# datafolder = joinpath(datadir,"labrun2"); camidxs =  0:5:1625
# datafolder = ENV["HOME"]*"/data/racecar/labrun3/"; # camidxs =
datafolder = ENV["HOME"]*"/data/racecar/labrun5/"; camidxs =  0:5:1020
# datafolder = ENV["HOME"]*"/data/racecar/labrun6/"; camidxs =  0:5:1795
# datafolder = ENV["HOME"]*"/data/racecar/labfull/"; camidxs =  0:5:1765
imgfolder = "images"
