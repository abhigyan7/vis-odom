cmake_minimum_required(VERSION 3.2)
project( slam )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Ceres REQUIRED )
include_directories( ${Ceres_INCLUDE_DIRS} )

set( CMAKE_BUILD_TYPE debug )

find_package( glog REQUIRED )
include_directories( ${glog_INCLUDE_DIRS} )

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package( GLEW REQUIRED )
if (GLEW_FOUND)
  include_directories($(GLEW_INCLUDE_DIRS))
endif()

include_directories("src")

file ( GLOB MAINSRCS src/main.cc )
file ( GLOB BASRCS src/bundle_adjustment_simple.cc )
add_executable( main ${MAINSRCS} )
add_executable( ba ${BASRCS} )

target_link_libraries( main ${OpenCV_LIBRARIES} )
target_link_libraries( main ${GLEW_LIBRARIES})
target_link_libraries( main ${Pangolin_LIBRARIES})
target_link_libraries( main ${GLOG_LIBRARIES})
target_link_libraries(main ${CERES_LIBRARIES})

target_link_libraries( ba ${OpenCV_LIBRARIES} )
target_link_libraries( ba ${CERES_LIBRARIES} )
message("Ceres: ${ceres_LIBRARIES}")

target_link_libraries( ba ${glog_LIBRARIES} )
target_link_libraries( ba glog::glog )
