cmake_minimum_required(VERSION 3.10)
project(xgc2_ros_display_relays VERSION 0.1.0 LANGUAGES CXX)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs geometry_msgs
  foxglove_msgs mavros_msgs std_msgs tf2_ros visualization_msgs xgc2_robot_visualization)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)
find_package(Threads REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES xgc2_ros_display_relays xgc2_ros_visualizer_contract
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs foxglove_msgs mavros_msgs std_msgs
  tf2_ros visualization_msgs xgc2_robot_visualization)
add_library(xgc2_ros_display_relays SHARED src/display_relays.cpp src/spec.cpp)
target_compile_features(xgc2_ros_display_relays PUBLIC cxx_std_14)
target_include_directories(xgc2_ros_display_relays PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS})
target_link_libraries(xgc2_ros_display_relays PUBLIC ${catkin_LIBRARIES}
  PRIVATE PkgConfig::JSONCPP Threads::Threads)
target_compile_options(xgc2_ros_display_relays PRIVATE -Wall -Wextra -Wpedantic)
add_executable(xgc2_display_relays src/node.cpp)
target_link_libraries(xgc2_display_relays PRIVATE xgc2_ros_display_relays)
install(TARGETS xgc2_ros_display_relays xgc2_display_relays
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_library(xgc2_ros_visualizer_contract SHARED src/xgc2_ros_visualizer_contract.cpp)
add_dependencies(xgc2_ros_visualizer_contract ${catkin_EXPORTED_TARGETS})
target_compile_features(xgc2_ros_visualizer_contract PUBLIC cxx_std_14)
target_include_directories(xgc2_ros_visualizer_contract PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  ${xgc2_robot_visualization_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(xgc2_ros_visualizer_contract PUBLIC ${catkin_LIBRARIES})
add_executable(xgc2_ros_visualizer_node src/xgc2_ros_visualizer_node.cpp)
add_dependencies(xgc2_ros_visualizer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(xgc2_ros_visualizer_node PRIVATE xgc2_ros_visualizer_contract)
install(TARGETS xgc2_ros_visualizer_contract xgc2_ros_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION lib/xgc2_ros_visualizer)
install(DIRECTORY include/xgc2_ros_visualizer/ DESTINATION include/xgc2_ros_visualizer)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(display_contract_test test/display_contract_test.cpp)
  if(TARGET display_contract_test)
    target_link_libraries(display_contract_test xgc2_ros_display_relays)
  endif()
  find_package(rostest REQUIRED)
  add_rostest_gtest(display_lifecycle_test test/display_lifecycle.test test/display_lifecycle_test.cpp)
  target_link_libraries(display_lifecycle_test xgc2_ros_display_relays)
  catkin_add_gtest(visualizer_contract_test test/visualizer_contract_test.cpp)
  if(TARGET visualizer_contract_test)
    target_link_libraries(visualizer_contract_test xgc2_ros_visualizer_contract ${catkin_LIBRARIES})
  endif()
endif()
