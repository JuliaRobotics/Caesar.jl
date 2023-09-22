module CaesarImagesExt

using Images
# using ImageTransformations
using ColorVectorSpace
using UUIDs
using TensorCast
using StaticArrays
using Manifolds
using DocStringExtensions
using ProgressMeter
using Optim
using JSON3
using Distributed

using Caesar # TODO try reduce to just import Caesar ... below
import Caesar._PCL as _PCL

# import DistributedFactorGraphs: getManifold

import Base: convert, show

import GeometricalPredicates as GeoPr

import ApproxManifoldProducts: sample, _update!
import IncrementalInference: getSample, preambleCache, _update!, getManifold

import Caesar: applyMaskImage, makeMaskImage, makeMaskImages, imhcatPretty
import Caesar: writevideo, csmAnimationJoinImgs, csmAnimateSideBySide, makeVideoFromData
import Caesar: overlayScanMatcher
import Caesar: overlayScatter, overlayScatterMutate

import Caesar: ScanMatcherPose2, PackedScanMatcherPose2
import Caesar: _PARCHABLE_PACKED_CLOUD
import Caesar: ScatterAlign, ScatterAlignPose2, ScatterAlignPose3
import Caesar: PackedScatterAlignPose2, PackedScatterAlignPose3

import Caesar: ImageTracks, FEATURE_VIA, FeatTrackValue, FeaturesDict, FeatureTracks, FeatureMountain, PIXELTRACK, MANYTRACKS
import Caesar: addFeatureTracks_Frame1_Q!, addFeatureTracks_Frame2_PfwdQ!, addFeatureTracks_Frame2_QbckR!
import Caesar: addFeatureTracks, consolidateFeatureTracks!, summarizeFeatureTracks!, buildFeatureMountain, buildFeatureMountainDistributed
import Caesar: unionFeatureMountain, sortKeysMinSighting

# NOTE usage requires ImageFeatures.jl
import Caesar: curateFeatureTracks


include("Images/ImageMask.jl")
include("Images/ImageToVideoUtils.jl")
include("Images/ScanMatcherUtils.jl")
include("Images/ScanMatcherPose2.jl")
include("Images/ScatterAlignPose2.jl")
include("Images/FeatureMountain.jl")


end # module