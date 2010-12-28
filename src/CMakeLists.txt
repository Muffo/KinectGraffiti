cmake_minimum_required(VERSION 2.8)

project(GraffitiDetection)
add_executable(graffiti graffiti.c libMyKinect.c libfreenect_sync.c)
find_package(OpenCV REQUIRED)
target_link_libraries(graffiti freenect ${OpenCV_LIBS})

install (TARGETS graffiti DESTINATION bin)
