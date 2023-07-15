
## ==============================================
# CaesarAprilTagsExt prototypes
export drawBearingLinesAprilTags!
export Pose2AprilTag4Corners, PackedPose2AprilTag4Corners
export generateCostAprilTagsPreimageCalib

function drawBearingLinesAprilTags! end


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

function overlayScatter end
function overlayScatterMutate end


## ==============================================
# CaesarImageFeaturesExt prototypes

export toDictFeatures
function toDictFeatures end


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
