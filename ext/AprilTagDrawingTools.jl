

# export drawBearingLinesAprilTags!


"""
    $SIGNATURES

Draw vertical lines where the tags are expected to be according to the factors.

```julia
imgs_ = deepcopy(imgs);

for i in 1:5
  @show fSym = intersect(lsf(fg, Pose2AprilTag4Corners), ls(fg, Symbol("x\$i")))
  for fct in getFactorType.(fg, fSym)
    drawBearingLinesAprilTags!( imgs_[i], fct,
                                f_width=f_width, c_width=c_width, taglength=taglength);
    #
  end
end

imshow( vcat(hcat( imgs_[1:3]... ), hcat( imgs_[4:6]... )) )
```
"""
function drawBearingLinesAprilTags!(img_::AbstractMatrix{<:AbstractRGB},
                                    atp4::Pose2AprilTag4Corners;
                                    f_width::Real=round(Int, size(img_,1)),
                                    f_height::Real=f_width,
                                    c_width::Real=round(Int, size(img_,2)/2),
                                    c_height::Real=round(Int, size(img_,1)/2),
                                    taglength::Real=0.172 )
  #
  xx,yy,th = (atp4.Z.Z.μ...,)

  # TODO use proper camera model
  py = round(Int, -f_width*(yy/xx) + c_width)
  ImageDraw.draw!(img_, ImageDraw.LineSegment(ImageDraw.Point(py,1), 
                                              ImageDraw.Point(py,size(img_,1))), RGB{N0f8}(1, 0, 0), 
                                              AprilTags.boundedBresenham)
end

function drawBearingLinesAprilTags!(img_::AbstractMatrix{<:AbstractRGB},
                                    tags::AbstractVector; 
                                    f_width::Real=round(Int, size(img_,1)),
                                    f_height::Real=f_width,
                                    c_width::Real=round(Int, size(img_,2)/2),
                                    c_height::Real=round(Int, size(img_,1)/2),
                                    taglength::Real=0.172 )
  #
  
  for tag_ in tags
    @show tag_.id
    drawBearingLinesAprilTags!( img_,
                                Pose2AprilTag4Corners( corners=tag_.p, homography=tag_.H, f_width=f_width, c_width=c_width, c_height=c_height, taglength=taglength),
                                f_width=f_width,
                                f_height=f_height,
                                c_width=c_width,
                                c_height=c_height,
                                taglength=taglength )
  end
  
  return img_
end

function drawBearingLinesAprilTags!(img_::AbstractMatrix{<:AbstractRGB}; 
                                    f_width::Real=round(Int, size(img_,1)),
                                    f_height::Real=f_width,
                                    c_width::Real=round(Int, size(img_,2)/2),
                                    c_height::Real=round(Int, size(img_,1)/2),
                                    taglength::Real=0.172  )
  #
  detector = AprilTagDetector()
  tags = deepcopy(detector(img_))
  
  res = drawBearingLinesAprilTags!( img_,
                                    tags; 
                                    f_width=f_width,
                                    f_height=f_height,
                                    c_width=c_width,
                                    c_height=c_height,
                                    taglength=taglength )
  #

  freeDetector!(detector)

  return res
end

