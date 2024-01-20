## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the
## TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 3.0.2)
project(rviz_plugin_tutorials)
find_package(catkin REQUIRED COMPONENTS rviz)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/src
)
link_directories(${catkin_LIBRARY_DIRS})


set(CMAKE_CXX_STANDARD 14)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets Gui)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets Qt5::Core Qt5::Gui)
endif()

find_package(Protobuf REQUIRED)
set(PROTO_PATH "${CMAKE_CURRENT_SOURCE_DIR}/proto")
file(GLOB PROTO_FILES "${PROTO_PATH}/*.proto")
# 使用protoc生成源码
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
# 包含生成的源码文件
include_directories(${CMAKE_CURRENT_BINARY_DIR})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
# add_definitions(-DQT_NO_KEYWORDS)

qt5_add_resources(RESOURCE_FILES resources.qrc)



## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  # src/drive_widget.cpp
  src/imu_display.cpp
  src/imu_visual.cpp
  src/plant_flag_tool.cpp
  src/teleop_panel.cpp
  src/grid_view2.cpp
  src/grid_display2.cpp
  # data_table
  src/data_table/filterwidget.cpp
  src/data_table/data_table_model.h
  src/data_table/data_table_widget.cpp
  src/data_table/data_table_display.cpp
  # plot 2d
  src/plot/qcustomplot.cpp
  src/plot/plot_base.cpp
  src/plot/trajectory_display.cpp
  src/plot/trajectory_widget.cpp
  src/plot/matrix_widget.cpp
  src/plot/matrix_display.cpp
  # sync
  src/time_sync.cpp
  src/display_sync_base.cpp
  src/time_sync_tool.cpp
  # proto
  src/protobuf_helper.cpp
  ${PROTO_SRCS} ${PROTO_HDRS}
  # properties
  src/properties/longitude_latitude_property.cpp
  # 
  ${RESOURCE_FILES}
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SRC_FILES}``.
# add_library(${PROJECT_NAME} ${SRC_FILES} ${MOC_SOURCES})
add_library(${PROJECT_NAME} ${SRC_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY})
## END_TUTORIAL

## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

catkin_install_python(PROGRAMS scripts/send_test_msgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
