## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds the
## TeleopPanel, ImuDisplay, and PlantFlagTool tutorials.

cmake_minimum_required(VERSION 3.8)
project(rviz_plugin_tutorials)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 
list(APPEND CMAKE_PREFIX_PATH "/home/gsk/install")
option(protobuf_MODULE_COMPATIBLE "123" ON)
find_package(protobuf REQUIRED)
find_package(fastrtps REQUIRED )
find_package(fastcdr REQUIRED )
find_package(glog REQUIRED)
find_package(cyber REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)


set(PROTO_PATH "${CMAKE_CURRENT_SOURCE_DIR}/proto")
file(GLOB PROTO_FILES "${PROTO_PATH}/*.proto")
# 使用protoc生成源码
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
# 包含生成的源码文件
include_directories(${CMAKE_CURRENT_BINARY_DIR})

qt5_add_resources(RESOURCE_FILES resources.qrc)


## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)


# # 添加消息文件
# set(msg_files
#   "msg/Wrapper.msg"
# )

# # 生成消息
# rosidl_generate_interfaces(${PROJECT_NAME}__rosidl_typesupport_cpp
#   ${msg_files}
#   DEPENDENCIES
#   std_msgs
# )



## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  # src/drive_widget.cpp
  src/plant_flag_tool.cpp
  src/teleop_panel.cpp
  # src/grid_view2.cpp
  # src/grid_display2.cpp
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
  src/plot/multi_matrix_display.cpp
  src/plot/precision_widget.cpp
  src/plot/precision_display.cpp
  # sync
  src/time_sync.cpp
  src/display_sync_base.cpp
  src/time_sync_tool.cpp
  src/my_3d_display.cpp
  # cacher
  src/cacher/cacher.cpp
  # proto
  src/protobuf_helper.cpp
  ${PROTO_SRCS} ${PROTO_HDRS}
  # properties
  src/properties/longitude_latitude_property.cpp
  src/properties/cached_channel_property.cpp
  src/properties/sub_plot_property.cpp
  src/properties/cyber_channel_property.cpp
  # 
  ${RESOURCE_FILES}
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SRC_FILES}``. We also add the needed dependencies.
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

# rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "rclcpp"
  "rviz_common"
  "rviz_rendering"
  "rviz_default_plugins"
  "sensor_msgs"
  "protobuf"
  # "${cpp_typesupport_target}"
  # "${sensor_msgs_TARGETS}"
  "cyber"
  "fastcdr"
  "fastrtps"
  "glog"
  )
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/src
  )
target_link_directories(${PROJECT_NAME} PUBLIC
  "/home/gsk/install/lib"
)

# target_link_libraries(${PROJECT_NAME}
# PUBLIC
#   cyber fastcdr fastrtps


# )

# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)


## Here we export the plugins and meshes so they can be found by RViz at runtime.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
register_rviz_ogre_media_exports(DIRECTORIES "media")

## END_TUTORIAL

ament_export_dependencies(
  geometry_msgs
  rclcpp
  rviz_common
  rviz_rendering
  sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons)

install(PROGRAMS scripts/send_test_msgs.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
