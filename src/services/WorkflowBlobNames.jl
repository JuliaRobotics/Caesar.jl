
ICPALIGNLOOPPRP(vla="",vlb="", suffix="") = "loopclosure_icp_$(vla)"*(vlb=="" ? "" : "_H_$(vlb)"*suffix)
SAPALIGNLOOPPRP(vla="",vlb="", suffix="") = "loopclosure_sap_$(vla)"*(vlb=="" ? "" : "_H_$(vlb)"*suffix)
ICPALIGNINGBLOB(vla="",vlb="", suffix="") = "homography_icp_$(vla)"*(vlb=="" ? "" : "_H_$(vlb)"*suffix)
SAPALIGNINGBLOB(vla="",vlb="", suffix="") = "homography_sap_$(vla)"*(vlb=="" ? "" : "_H_$(vlb)"*suffix)
GYRODELTAROTATE(vla="",vlb="")    = "Gyro_DeltaRotation_$(vla)"*(vlb=="" ? "" : "_R_$(vlb)")
IMUDATABLOBNAME(seqa="", seqb="") = "IMU_$seqa"*(seqb=="" ? "" : "_$(seqb)")
ODOALIGNMENTREP() = "OdoAlignmentReport"
CALIBLIDARTOIMU() = "calib_lidar_to_imu"
MAPEXPORTBAGVIZ(robotId, sessionId) = "MapBagExport_$(robotId)_$(sessionId)"
CAMERA_BLOBNAME() = "CameraImage"
CAMINFO_BLOBNAME() = "CameraInfo"
TF2MSG_BLOBNAME() = "TransformReference"
ODOMSG_BLOBNAME() = "OdometryData"
GNSS_BLOB_LABEL(;suffix="") = "GNSS_GPS_Fix"*suffix
TWISTCOV_BLOB_LABEL(;suffix="") = "TWISTCOV_MSG"*suffix
RADARPOINTCLOUD(;suffix="") = "RadarPCLPointCloud2"*suffix

IMU_DATA_BLOBNAME(suffix) = "IMU_"*suffix
SATNAV_DATA_BLOBNAME(suffix) = "SATNAV_"*suffix
VEHICLE_DATA_BLOBNAME(suffix) = "VEHICLE_"*suffix
CAMERA_RAW_BLOBNAME(suffix) = "CAM_RAW_"*suffix
CAMERA_CAL_BLOBNAME(suffix) = "CAM_CAL_"*suffix

#