CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(KinectProject)

IF(APPLE)
   #SET(GUI_TYPE MACOSX_BUNDLE)
   INCLUDE_DIRECTORIES ( /Developer/Headers/FlatCarbon )
   FIND_LIBRARY(APP_SERVICES_LIBRARY ApplicationServices )
   MARK_AS_ADVANCED (APP_SERVICES_LIBRARY)
   SET(EXTRA_LIBS ${APP_SERVICES_LIBRARY})
ENDIF (APPLE)


ADD_EXECUTABLE(graffiti graffiti.c libfreenect_sync.c libMyKinect.c)
ADD_EXECUTABLE(hand hand.c libfreenect_sync.c libMyKinect.c  eig3.c libSkin.c gestureManager.c libMouseOSX.c)

FIND_PACKAGE(OpenCV REQUIRED)
TARGET_LINK_LIBRARIES(graffiti freenect ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(hand freenect ${OpenCV_LIBS} ${EXTRA_LIBS})

INSTALL (TARGETS graffiti DESTINATION bin)
INSTALL (TARGETS hand DESTINATION bin)

