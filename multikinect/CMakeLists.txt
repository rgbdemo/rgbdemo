QT4_WRAP_CPP(moc_sources
  GuiController.h
  View3dWindow.h
  FiltersWindow.h
  RawImagesWindow.h
)

QT4_WRAP_UI(uic_sources
  RawImagesWindow.ui
  View3dWindow.ui
  FiltersWindow.ui
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

SET(rgbd_multikinect_sources ${moc_sources} ${uic_sources}
               main.cpp
               GuiController.cpp GuiController.h
               View3dWindow.cpp View3dWindow.h
               FiltersWindow.cpp FiltersWindow.h
               RawImagesWindow.cpp RawImagesWindow.h
)

IF (BUILD_MACOSX_BUNDLE)
   ADD_EXECUTABLE(rgbd-multikinect MACOSX_BUNDLE ${rgbd_multikinect_sources})
   SET(DIRS ${QT_LIBRARY_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/bin)
   INSTALL_STANDALONE_BUNDLE(rgbd-multikinect ${DIRS})
ELSE()
   ADD_EXECUTABLE(rgbd-multikinect ${rgbd_multikinect_sources})
ENDIF()

TARGET_LINK_LIBRARIES(rgbd-multikinect
                      nestk)

INSTALL(TARGETS rgbd-multikinect DESTINATION bin)
