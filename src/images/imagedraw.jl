
@info "Caesar.jl is loading tools using ImageDraw.jl."

using .ImageDraw
using .Colors

export makeImage!


function makeImage!(pc::Caesar._PCL.PointCloud,
                    x_domain::Tuple{<:Real,<:Real}=(-1000,1000),
                    y_domain::Tuple{<:Real,<:Real}=x_domain;
                    rows::Integer=1000, 
                    cols::Integer=rows,
                    img::AbstractMatrix{<:Colorant} = Gray.(zeros(UInt8,rows,cols)),
                    circle_size::Real=1,
                    drawkws... )
  #

  x_range = (x_domain[2]-x_domain[1])
  y_range = (y_domain[2]-y_domain[1])
  gridsize_x = x_range/rows
  gridsize_y = y_range/cols
  
  for pt in pc.points
    _x = trunc(Int, pt.x/gridsize_x + rows/2 )
    _y = trunc(Int, pt.y/gridsize_y + cols/2 )
    draw!( img, Ellipse(CirclePointRadius(_x,_y, circle_size)); drawkws... )  #; thickness = 1, fill = true)) )
  end
  
  return img
end
