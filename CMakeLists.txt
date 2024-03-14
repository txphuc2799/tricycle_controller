cmake_minimum_required(VERSION 3.0.2)
project(tricycle_controller)


# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  controller_interface
  urdf
  realtime_tools
  hardware_interface
  tf
  nav_msgs
  pluginlib
  geometry_msgs
)

find_package(Boost REQUIRED)

find_package(urdfdom REQUIRED)

# Declare a catkin package
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    nav_msgs
    realtime_tools
    roscpp
    tf
)

###########
## Build ##
###########

# Specify header include paths
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/tricycle_controller.cpp src/odometry.cpp src/steering_limiter.cpp src/traction_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES})

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Install plugins
install(FILES tricycle_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})