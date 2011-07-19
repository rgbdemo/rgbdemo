SET(common_sources calibration_common.h calibration_common.cpp)

ADD_EXECUTABLE(annotate_image annotate_image.cpp ${common_sources})
TARGET_LINK_LIBRARIES(annotate_image nestk)
#INSTALL(TARGETS annotate_image DESTINATION bin)

ADD_EXECUTABLE(calibrate_kinect calibrate_kinect.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate_kinect nestk)
#INSTALL(TARGETS calibrate_kinect DESTINATION bin)

ADD_EXECUTABLE(calibrate-multiple-kinects calibrate_multiple_kinects.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-multiple-kinects nestk)

ADD_EXECUTABLE(calibrate-openni-intrinsics calibrate_openni_intrinsics.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-openni-intrinsics nestk)

ADD_EXECUTABLE(calibrate-openni-depth calibrate_openni_depth.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-openni-depth nestk)

IF (BUILD_MACOSX_BUNDLE)
   ADD_EXECUTABLE(calibrate_kinect_ir MACOSX_BUNDLE calibrate_kinect_ir.cpp ${common_sources})
   SET(DIRS ${QT_LIBRARY_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/bin)
   INSTALL_STANDALONE_BUNDLE(calibrate_kinect_ir ${DIRS})
ELSE()
   ADD_EXECUTABLE(calibrate_kinect_ir calibrate_kinect_ir.cpp ${common_sources})
ENDIF()

TARGET_LINK_LIBRARIES(calibrate_kinect_ir nestk)
INSTALL(TARGETS calibrate_kinect_ir DESTINATION bin)
