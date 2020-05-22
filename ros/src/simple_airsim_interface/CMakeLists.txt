cmake_minimum_required(VERSION 2.8.3)

project(simple_airsim_interface)

# Specify all of the pacakge dependencies here. Look in package.xml
# for the dependencies that should be listed here
set(DEPS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs)

### Catkin ###
find_package(catkin
  REQUIRED
  COMPONENTS
    ${DEPS})

catkin_package(
  CATKIN_DEPENDS ${DEPS}
  INCLUDE_DIRS include)

# Find several common OpenCV components, even though we do not
# use them in this example
find_package(OpenCV
  REQUIRED
  COMPONENTS
    core
    imgproc
    highgui)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

### Build C++ nodes ###
add_executable(airsim_cpp_example
  src/airsim_cpp_example.cpp)
target_link_libraries(airsim_cpp_example
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

### Install Libraries and Executables ###
install(TARGETS airsim_cpp_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

### Install Header Files ###
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

### Install Launch Files ###
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

### Install Python Scripts ###
catkin_install_python(PROGRAMS nodes/airsim_example
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})