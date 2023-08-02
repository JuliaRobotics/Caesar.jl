
## ==============================================
# CaesarAprilTagsExt prototypes
export drawBearingLinesAprilTags!
export Pose2AprilTag4Corners, PackedPose2AprilTag4Corners
export generateCostAprilTagsPreimageCalib

function drawBearingLinesAprilTags! end
function generateCostAprilTagsPreimageCalib end

## ==============================================
# CaesarImagesExt prototypes

export applyMaskImage
export makeMaskImage
export makeMaskImages
export imhcatPretty
export toImage
export writevideo, csmAnimationJoinImgs, csmAnimateSideBySide
export makeVideoFromData

export ScanMatcherPose2, PackedScanMatcherPose2
export ScatterAlignPose2, PackedScatterAlignPose2
export ScatterAlignPose3, PackedScatterAlignPose3

export overlayScanMatcher
export overlayScanMatcherOLD
export overlayScatter, overlayScatterMutate


function applyMaskImage end
function makeMaskImage end
function makeMaskImages end
function imhcatPretty end
function toImage end

function writevideo end
function csmAnimationJoinImgs end
function csmAnimateSideBySide end
function makeVideoFromData end

function overlayScanMatcher end
function overlayScanMatcherOLD end

function overlayScatter end
function overlayScatterMutate end


#experimental features

const ImageTracks = Dict{Int, Vector{Vector{Float32}}}
# features[nframes][track_uuid][img_id] = featidx,pixeluv   # image feature used in multiple frame counts, 2 / 3 / 4 / ... each with own track_uuid
# const FeaturesDict = Dict{Pair{Symbol,Symbol}, Tuple{Int,Tuple{Float64,Float64}}}
const FEATURE_VIA = Tuple{Symbol, Int64, Tuple{Float64, Float64}}
const FeatTrackValue = NamedTuple{(:from, :to), Tuple{FEATURE_VIA, FEATURE_VIA}}
# Tuple{Tuple{Float64,Float64}, Pair{Tuple{Symbol,Int}, Tuple{Symbol,Int}}}
const FeaturesDict = Dict{Symbol,FeatTrackValue}
const FeatureTracks = Dict{UUID, FeaturesDict}
const FeatureMountain = Dict{Int, FeatureTracks}
# NOTE # images[img_id][track_uuid][pixel] # this format doesnt work because same feature my occur in multiple frame counts

const PIXELTRACK = NamedTuple{(:via, :track), Tuple{FEATURE_VIA, Tuple{UUID,Tuple{Symbol,Int},Tuple{Float64,Float64}}}}
const MANYTRACKS = Dict{Tuple{Symbol,Int},PIXELTRACK}


function addFeatureTracks_Frame1_Q! end
function addFeatureTracks_Frame2_PfwdQ! end
function addFeatureTracks_Frame2_QbckR! end
function addFeatureTracks end
function consolidateFeatureTracks! end
function summarizeFeatureTracks! end
function buildFeatureMountain end


## ==============================================
# CaesarImageFeaturesExt prototypes

export toDictFeatures
function toDictFeatures end

function distancesKeypoints end
function sortMinimumsToDiagonal end
function addDataImgTracksFwdBck! end
function curateFeatureTracks end

## ==============================================
# CaesarImageMagickExt prototypes
export toFormat
export fetchDataImage

function toFormat end
function fetchDataImage end


## ==============================================
# CaesarZMQExt prototypes
export
  # addVariable,
  # addFactor,
  addOdometry2D,
  addLandmark2D,
  addFactorBearingRangeNormal
  # ls,
  # getVert,
  # setSolvable,
  # solveTree!,
  # per variable
  # setVarKDE, # needed for workaround on bad autoinit
  # getVarMAPKDE, # marginal belief points (KDE)
  # getVarMAPMax, # Future, how many maxes should you get?
  # getVarMAPMean,
  # # fancy future stuff
  # getVarMAPFit # defaul=Normal


function addOdometry2D end
function addLandmark2D end
function addFactorBearingRangeNormal end
