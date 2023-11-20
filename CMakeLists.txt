cmake_minimum_required(VERSION 3.5)
include(ExternalProject)
project(slic3r_coverage_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
        

set(EP_SLIC3R "Slic3r")
ExternalProject_Add(
        ${EP_SLIC3R}

        PREFIX ${EP_SLIC3R}
        GIT_REPOSITORY https://github.com/HoverMower/Slic3r
        #GIT_TAG 026c1380e0ad12176dd2fc8cdf8f6f181deeb071
        GIT_SHALLOW OFF


        BUILD_ALWAYS OFF

        INSTALL_COMMAND cmake -E echo "Skipping install step."

        CMAKE_CACHE_ARGS
        SOURCE_SUBDIR src
        CMAKE_ARGS -DSLIC3R_BUILD_TESTS=OFF
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release
)



## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/Path.msg"
   "srv/PlanPath.srv"
   DEPENDENCIES
   geometry_msgs
   nav_msgs 
 )


###########
## Build ##
###########
include_directories(
  include
  include/slic3r_coverage_planner
  include ${Boost_INCLUDE_DIRS}
)



ExternalProject_Get_property(${EP_SLIC3R} SOURCE_DIR)
message("Source dir of ${EP_SLIC3R} = ${SOURCE_DIR}")
include_directories(${SOURCE_DIR}/xs/src/libslic3r)
include_directories(${SOURCE_DIR}/xs/src)


add_executable(coverage_planner
        src/coverage_planner.cpp
        src/main.cpp
        src/OutlinePoly.cpp
        )

add_dependencies(slic3r_coverage_planner ${EP_SLIC3R})


ExternalProject_Get_property(${EP_SLIC3R} BINARY_DIR)
target_link_libraries(coverage_planner #${catkin_LIBRARIES}
        ${BINARY_DIR}/liblibslic3r.a
        ${BINARY_DIR}/libadmesh.a
        ${BINARY_DIR}/libBSpline.a
        ${BINARY_DIR}/libexpat.a
        ${BINARY_DIR}/libminiz.a
        ${BINARY_DIR}/libpoly2tri.a
        ${BINARY_DIR}/libpolypartition.a
        ${BINARY_DIR}/libZipArchive.a
        ${BINARY_DIR}/libclipper.a
        ${Boost_LIBRARIES}
)

ament_target_dependencies(coverage_planner rclcpp
  tf2
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  )

  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(coverage_planner "${cpp_typesupport_target}")

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS coverage_planner
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.h"
#)

install(TARGETS
  coverage_planner
  DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(geometry_msgs)
ament_package()
