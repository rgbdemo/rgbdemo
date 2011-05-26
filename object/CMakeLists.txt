QT4_WRAP_CPP(moc_sources
  GuiController.h
  ModelAcquisitionController.h
  View3dWindow.h
  FiltersWindow.h
  RawImagesWindow.h
  ModelAcquisitionWindow.h
)

QT4_WRAP_UI(uic_sources
  RawImagesWindow.ui
  View3dWindow.ui
  FiltersWindow.ui
  ModelAcquisitionWindow.ui
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
SET(rgbd_object_sources ${moc_sources} ${uic_sources}
               main.cpp
               GuiController.cpp GuiController.h
               ModelAcquisitionController.cpp ModelAcquisitionController.h
               View3dWindow.cpp View3dWindow.h
               FiltersWindow.cpp FiltersWindow.h
               RawImagesWindow.cpp RawImagesWindow.h
               ModelAcquisitionWindow.cpp ModelAcquisitionWindow.h
)

IF (BUILD_MACOSX_BUNDLE)
   ADD_EXECUTABLE(rgbd-object ${rgbd_object_sources})
   SET(DIRS ${QT_LIBRARY_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/bin)
   INSTALL_STANDALONE_BUNDLE(rgbd-object ${DIRS})
ELSE()
   ADD_EXECUTABLE(rgbd-object ${rgbd_object_sources})
ENDIF()

TARGET_LINK_LIBRARIES(rgbd-object
                      nestk)
