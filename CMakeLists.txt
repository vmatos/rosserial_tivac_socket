cmake_minimum_required(VERSION 2.8.3)
project(rosserial_tivac_socket)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS rosserial_tivac_socket-extras.cmake
)

install(
  DIRECTORY
    src/ros_lib
    src/ros_lib_energia
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(
  DIRECTORY tivac-cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS 
    src/rosserial_tivac_socket/make_libraries_tiva
    src/rosserial_tivac_socket/make_libraries_energia
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
