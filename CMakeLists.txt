cmake_minimum_required(VERSION 2.8.3)
project(vslam)
#SET( CMAKE_CXX_COMPILER "g++") #设定编译器

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
image_transport
cv_bridge)

# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)

# 增加opencv的依赖
FIND_PACKAGE( OpenCV 2.4 REQUIRED )





###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  CATKIN_DEPENDS sensor_msgs roscpp std_msgs image_transport cv_bridge
  DEPENDS PCL OpenCV
  LIBRARIES SlamBase
)


#catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES SlamBase
  #CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )   


ADD_LIBRARY(SlamBase src/SlamBase/SlamBase.cpp)
TARGET_LINK_LIBRARIES( SlamBase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
)
add_dependencies(SlamBase ${catkin_EXPORTED_TARGETS})


ADD_EXECUTABLE(VSlam src/VSlam/VSlam.cpp)
TARGET_LINK_LIBRARIES( VSlam
    SlamBase
${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
${catkin_LIBRARIES})
add_dependencies(VSlam ${catkin_EXPORTED_TARGETS})

#ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src/SlamBase)
#ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src/VSlam)



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/vslam.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/vslam_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

