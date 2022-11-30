cmake_minimum_required(VERSION 3.5.1)
project(quadtree_ros)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 
find_package(catkin REQUIRED cmake_modules)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge

  geometry_msgs
  sensor_msgs
  
  image_transport
)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
)


set(SRC ${SRC} 
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

# executable for 'scale_mono_vo' node
add_executable(quadtree_node src/main.cpp ${SRC})
target_link_libraries (quadtree_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
