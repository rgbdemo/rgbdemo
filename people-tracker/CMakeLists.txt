QT4_WRAP_CPP(moc_sources
  GuiController.h
  FiltersWindow.h
  RawImagesWindow.h
  PeopleTrackerWindow.h
)

QT4_WRAP_UI(uic_sources
  RawImagesWindow.ui
  FiltersWindow.ui
  PeopleTrackerWindow.ui
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

ADD_EXECUTABLE(rgbd-people-tracker ${moc_sources} ${uic_sources}
               people_tracker.cpp
               GuiController.cpp GuiController.h
               FiltersWindow.cpp FiltersWindow.h
               RawImagesWindow.cpp RawImagesWindow.h
               PeopleTracker.cpp PeopleTracker.h
               PeopleTrackerWindow.cpp PeopleTrackerWindow.h
)

TARGET_LINK_LIBRARIES(rgbd-people-tracker
                      nestk)

INSTALL(TARGETS rgbd-people-tracker DESTINATION bin)
