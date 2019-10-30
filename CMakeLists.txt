cmake_minimum_required(VERSION 2.8.3)
project(ground_detect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp std_msgs rospy  sensor_msgs pcl_ros pcl_msgs   tf tf_conversions  nav_msgs message_filters
)
# message_filters

find_package(catkin REQUIRED COMPONENTS
  message_generation
   ${${PROJECT_NAME}_CATKIN_DEPS} )

#show include
# message(STATUS "localization include_directories:" )
# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${dirs})
#   message(STATUS "dir='${dir}'")
# endforeach()

find_package(PCL 1.7 REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

add_message_files(FILES
        road_range.msg
        )

generate_messages(DEPENDENCIES
        std_msgs
        )

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS} message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(EIGEN_INCLUDE_PATH "/usr/include/eigen3")
include_directories(
  include
  ${EIGEN_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
)


# #node
#  add_executable(blank
#          src/blank.cpp
#          )
#  add_dependencies(blank ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#  target_link_libraries(blank
#          ${catkin_LIBRARIES}
#          ${PCL_LIBRARIES}
#          )

add_executable(lidar_line_analyse
  src/lidar_line_analyse.cpp
)
add_dependencies(lidar_line_analyse ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_line_analyse
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(feature_based_select
  src/feature_based_select.cpp
)
add_dependencies(feature_based_select ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(feature_based_select
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(local_env_provider
        src/local_env_provider.cpp
        )
add_dependencies(local_env_provider ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_env_provider
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(grid_map_analyser
        src/grid_map_analyser.cpp
        )
add_dependencies(grid_map_analyser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grid_map_analyser
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )


#install node
# install(TARGETS streeing_to_motion
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# #install library http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS rt_localization_nodelet globalmap_provider_nodelet
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#         PATTERN ".svn" EXCLUDE)

# #install launch
# install(DIRECTORY launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#         )
