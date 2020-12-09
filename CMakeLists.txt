cmake_minimum_required(VERSION 2.8.3)
project(radar_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package( OpenCV REQUIRED )
find_package( X11 REQUIRED)
find_package( ZLIB REQUIRED )

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH}
          /usr/local/SPx/current/Libs/linux)

find_library(IMF NAMES libimf.a PATHS CMAKE_LIBRARY_PATH STATIC IMPORTED)
find_library(IRC NAMES libirc.a PATHS CMAKE_LIBRARY_PATH STATIC IMPORTED)
find_library(MXLIN260 NAMES libmxlin260.a PATHS CMAKE_LIBRARY_PATH STATIC IMPORTED)
find_library(MXLIN260_64 NAMES libmxlin260_64.a PATHS CMAKE_LIBRARY_PATH STATIC IMPORTED)
find_library(SPX NAMES libspx.a PATHS CMAKE_LIBRARY_PATH STATIC IMPORTED)
find_library(SPX_64 NAMES libspx_64.a PATHS CMAKE_LIBRARY_PATH STATIC IMPORTED)
find_library(SPXQT NAMES libspxqt.a PATHS CMAKE_LIBRARY_PATH STATIC IMPORTED)
find_library(SPXQT_5 NAMES libspxqt_5.a PATHS CMAKE_LIBRARY_PATH STATIC IMPORTED)
find_library(SPXQT_5_64 NAMES libspxqt_5_64.a PATHS CMAKE_LIBRARY_PATH STATIC IMPORTED)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  /usr/local/SPx/current/Includes
  ${catkin_INCLUDE_DIRS}
  ${ZLIB_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)

add_library(SPxServerScanImgProcess src/SPxServerScanImgProcess.cpp)
add_dependencies(SPxServerScanImgProcess ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(SPxServerScanImgProcess ${catkin_LIBRARIES})


add_executable(radar_monitor_node src/spxppi1.cpp)
add_dependencies(radar_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_monitor_node ${catkin_LIBRARIES}
#SPxServerScanImgProcess
${OpenCV_LIBS} 
/usr/local/SPx/current/Libs/linux/libspx_64.a 
/usr/local/SPx/current/Libs/linux/libmxlin260_64.a
${ZLIB_LIBRARIES} ${X11_LIBRARIES} Xext Xrender ${X11_Xau_LIB}
usb pthread
Xcomposite X11 Xt Xaw
)

add_executable(radar_rospub_node src/spxppi_no_gui.cpp)
add_dependencies(radar_rospub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_rospub_node ${catkin_LIBRARIES}
/usr/local/SPx/current/Libs/linux/libspx_64.a
/usr/local/SPx/current/Libs/linux/libmxlin260_64.a
usb pthread z)
