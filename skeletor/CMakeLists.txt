QT4_WRAP_CPP(moc_sources
  GuiController.h
  RawImagesWindow.h
)

QT4_WRAP_UI(uic_sources
  RawImagesWindow.ui
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

SET(rgbd_skeletor_sources ${moc_sources} ${uic_sources}
               skeletor.cpp
               RawImagesWindow.h RawImagesWindow.cpp
               GuiController.cpp GuiController.h)

NESTK_ADD_EXECUTABLE(rgbd-skeletor ${rgbd_skeletor_sources})
TARGET_LINK_LIBRARIES(rgbd-skeletor nestk)
