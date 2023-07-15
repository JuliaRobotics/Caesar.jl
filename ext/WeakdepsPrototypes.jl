
## ==============================================
# CaesarAprilTagsExt prototypes
export drawBearingLinesAprilTags!
export Pose2AprilTag4Corners, PackedPose2AprilTag4Corners
export generateCostAprilTagsPreimageCalib

function drawBearingLinesAprilTags! end


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
