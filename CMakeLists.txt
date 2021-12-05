cmake_minimum_required(VERSION 3.2)
project( slam )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Ceres REQUIRED )
include_directories( ${Ceres_INCLUDE_DIRS} )

find_package( glog REQUIRED )
include_directories( ${glog_INCLUDE_DIRS} )

add_subdirectory("polyscope")


file ( GLOB MAINSRCS src/main.cc )
file ( GLOB BASRCS src/bundle_adjustment_simple.cc )
add_executable( main ${MAINSRCS} )
add_executable( ba ${BASRCS} )

target_link_libraries( main ${OpenCV_LIBRARIES} )
target_link_libraries( main polyscope )

target_link_libraries( ba ${OpenCV_LIBRARIES} )
target_link_libraries( ba ${CERES_LIBRARIES} )
message("Ceres: ${ceres_LIBRARIES}")

target_link_libraries( ba ${glog_LIBRARIES} )
target_link_libraries( ba glog::glog )
target_link_libraries( ba polyscope )
target_link_libraries( ba polyscope )
