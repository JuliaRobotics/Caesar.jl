# stitch images and estimate together

using ImageMagick
# using ImageShow
using Images, ImageView, ImageDraw


datadir = joinpath(ENV["HOME"],"data","racecar")
resultsparentdir = joinpath(datadir, "results")
imgfolder = "images"

currdirtime = "2018-11-07T01:36:52.274"
resultsdir = joinpath(resultsparentdir, "$(currdirtime)")

i = 1
for i in 1:10

tag_img = load( joinpath(resultsdir, imgfolder, "x$(i).jpg") )
top_img = load( joinpath(resultsdir, imgfolder, "img_$(i).png") )

rows = size(im1)[1]+size(im1t)[1]

end



#
