QT4_WRAP_CPP(moc_sources
  GuiController.h
  ModelAcquisitionController.h
  DetectorWindow.h
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
  DetectorWindow.ui
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

ADD_EXECUTABLE(rgbd-viewer ${moc_sources} ${uic_sources}
               demo3d.cpp
               GuiController.cpp GuiController.h
               ModelAcquisitionController.cpp ModelAcquisitionController.h
               ObjectDetector.cpp ObjectDetector.h
               View3dWindow.cpp View3dWindow.h
               FiltersWindow.cpp FiltersWindow.h
               DetectorWindow.cpp DetectorWindow.h
               RawImagesWindow.cpp RawImagesWindow.h
               ModelAcquisitionWindow.cpp ModelAcquisitionWindow.h
)

TARGET_LINK_LIBRARIES(rgbd-viewer
                      nestk)
