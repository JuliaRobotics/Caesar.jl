module CaesarAprilTagsExt

@info "Caesar is loading tools related to AprilTags.jl."

using AprilTags
using ImageDraw
using DocStringExtensions
using Colors

using RoME

import Base: getproperty, propertynames

import IncrementalInference: getSample, AbstractRelativeMinimize, getManifold

# entities
import Caesar: Pose2AprilTag4Corners, PackedPose2AprilTag4Corners
# services
import Caesar: drawBearingLinesAprilTags!

include("AprilTagDrawingTools.jl")
include("Pose2AprilTag4Corners.jl")


end