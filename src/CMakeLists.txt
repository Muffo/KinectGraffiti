cmake_minimum_required(VERSION 2.8)

project(KinectProject)
add_executable(graffiti graffiti.c libMyKinect.c libfreenect_sync.c)
add_executable(hand hand.c libMyKinect.c eig3.c libfreenect_sync.c)

find_package(OpenCV REQUIRED)
target_link_libraries(graffiti freenect ${OpenCV_LIBS})
target_link_libraries(hand freenect ${OpenCV_LIBS})

install (TARGETS graffiti DESTINATION bin)
install (TARGETS hand DESTINATION bin)

