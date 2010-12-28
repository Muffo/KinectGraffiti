cmake_minimum_required(VERSION 2.8)

project(GraffitiDetection)
add_executable(graffiti graffiti.c libfreenect_sync.c libMyKinect.c)
find_package(OpenCV REQUIRED)
target_link_libraries(graffiti freenect ${OpenCV_LIBS})

install (TARGETS graffiti DESTINATION bin)
