

# somehow get all AT detections
# you get some kind of list/dict of AT detections per pose
for p in poses
  # p is symbol such as :x45
  for detection in dets[p]
    # detection has (id, homography)
    lmsyms = ls(fg)[2]
    atsym = Symbol("l$(detection.id)")
    if !(atsym in lmsyms)
      addVariable!(fg, atsym, Point2(tags=[:LANDMARK; :APRILTAG]))
    end

    # reverse homography to range, bearing
    pp2 = Point2Point2BearingRange(...)
    addFactor!(fg, [p; atsym], pp2)
  end
end
