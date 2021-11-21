
@info "Caesar.jl is loading tools using ImageDraw.jl."

using .ImageDraw
using .Colors

export makeImage!


function makeImage!(pc::Caesar._PCL.PointCloud,
                    x_domain::Tuple{<:Real,<:Real}=(-1000,1000),
                    y_domain::Tuple{<:Real,<:Real}=x_domain;
                    pose=nothing,
                    ppose=nothing,
                    rows::Integer=1000, 
                    cols::Integer=rows,
                    color::C=Gray(0.1),
                    trajCol=Gray(1.0),
                    img::AbstractMatrix{<:Colorant} = Gray.(zeros(rows,cols)),
                    circle_size::Real=1,
                    drawkws... ) where {C <: Colorant}
  #

  x_range = (x_domain[2]-x_domain[1])
  y_range = (y_domain[2]-y_domain[1])
  gridsize_x = x_range/rows
  gridsize_y = y_range/cols
  
  for pt in pc.points
    _x = trunc(Int, ( pt.x-x_domain[1])/gridsize_x ) #  + rows/2
    _y = trunc(Int, (-pt.y-y_domain[1])/gridsize_y ) #  + cols/2
    draw!( img, Ellipse(CirclePointRadius(_x, _y, circle_size)), color; drawkws... )  #; thickness = 1, fill = true)) )
  end

  # draw pose location
  if !(pose isa Nothing)
    _x = trunc(Int, ( pose[1]-x_domain[1])/gridsize_x ) #  + rows/2
    _y = trunc(Int, (-pose[2]-y_domain[1])/gridsize_y ) #  + cols/2

    draw!( img, Ellipse(CirclePointRadius(_x, _y, 4)), trajCol )

    if !(ppose isa Nothing)
      __x = trunc(Int, ( pose[1]-x_domain[1])/gridsize_x ) #  + rows/2
      __y = trunc(Int, (-pose[2]-y_domain[1])/gridsize_y ) #  + cols/2

      draw!( img, LineSegment(Point(_x, _y), Point(__x, __y)), trajCol )
    end
  end
  
  return img
end
