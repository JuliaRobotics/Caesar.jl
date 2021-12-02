# interactive scan match test

using GLMakie
using Images, ImageTransformations
using ImageView

##

# arp = AlignRadarPose2(...)

##

fig = Figure()
ax = GLMakie.Axis(fig[1,1])
guidict = ImageView.imshow(overlayScanMatcher(arp, [0;0], 0))
canvas = guidict["gui"]["canvas"]

guidict2 = imshow(arp.im1 - arp.im2)
canvas2 = guidict2["gui"]["canvas"]


sl_x = Slider(fig[2, 1], range = -40:1:40, startvalue = 0)
sl_r = Slider(fig[3, 1], range = (-pi/2):0.01:(pi/2), startvalue = 0)
sl_y = Slider(fig[1, 2], range = -40:1:40, horizontal = false, startvalue = 0)

score = Ref(0.0)

##

lift(sl_x.value, sl_y.value, sl_r.value) do x, y, r
  im_ = overlayScanMatcher(arp, [x;y], r; score)
  println("Making x,y,r = $x,$y,$r = $(score[])")
  ImageView.imshow(canvas, im_)

  a = arp.im1
  b = arp.im2
  bp = Caesar.transformImage_SE2(b, [x;y], r, arp.gridscale)
  # get matching padded views
  ap, bpp = paddedviews(0.0, a, bp)
  # @show sqrt(sum((ap-bpp).^2))
  ImageView.imshow(canvas2, ap-bpp)
end

fig

##


#