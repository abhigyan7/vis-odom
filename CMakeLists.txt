cmake_minimum_required(VERSION 3.2)
project( slam )
set( CMAKE_BUILD_TYPE debug )

find_package( Ceres REQUIRED )
find_package( OpenCV REQUIRED )
find_package( glog REQUIRED )
find_package(Pangolin REQUIRED)
find_package( GLEW REQUIRED )

include_directories( ${Ceres_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${glog_INCLUDE_DIRS} )
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories($(GLEW_INCLUDE_DIRS))


include_directories("inc")
file ( GLOB MAINSRCS src/*.cc )
add_executable( main ${MAINSRCS} )

target_link_libraries(main ${CERES_LIBRARIES})
target_link_libraries( main ${OpenCV_LIBRARIES} )
target_link_libraries( main ${GLOG_LIBRARIES})
target_link_libraries( main ${Pangolin_LIBRARIES})
target_link_libraries( main ${GLEW_LIBRARIES})
