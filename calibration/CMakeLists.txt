SET(common_sources calibration_common.h calibration_common.cpp)

ADD_EXECUTABLE(annotate_image annotate_image.cpp ${common_sources})
TARGET_LINK_LIBRARIES(annotate_image nestk)

ADD_EXECUTABLE(calibrate_kinect calibrate_kinect.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate_kinect nestk)

ADD_EXECUTABLE(calibrate_kinect_ir calibrate_kinect_ir.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate_kinect_ir nestk)
