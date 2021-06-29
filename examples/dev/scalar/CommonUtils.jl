module CommonUtils

using Caesar
using Images
using FileIO

export getSampleDEM

function getSampleDEM(scale=1)
    img = load(joinpath(dirname(dirname(pathof(Caesar))), "examples","dev","scalar","dem.png")) .|> Gray
    img = scale.*Float64.(img)
    x = range(-9e3, 9e3, length = size(img)[1]) # North
    y = range(-9e3, 9e3, length = size(img)[2]) # East
    return (x, y, img)
end

end