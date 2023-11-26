cmake_minimum_required(VERSION 3.16)
project(optik_kinematics_plugin)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

include(FetchContent)
FetchContent_Declare(
  optik
  GIT_REPOSITORY https://github.com/kylc/optik
  GIT_TAG kin-plugin # TODO
  SOURCE_SUBDIR crates/optik-cpp)
FetchContent_MakeAvailable(optik)

add_library(optik_kinematics_plugin SHARED src/optik_kinematics_plugin.cpp)
target_compile_features(optik_kinematics_plugin PUBLIC cxx_std_11)
target_include_directories(
  optik_kinematics_plugin
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/optik_kinematics_plugin>)
target_link_libraries(
  optik_kinematics_plugin
  PUBLIC moveit_core::moveit_robot_model tf2_geometry_msgs::tf2_geometry_msgs
  PRIVATE moveit_core::moveit_kinematics_base moveit_core::moveit_robot_state
          pluginlib::pluginlib rclcpp::rclcpp tf2_eigen::tf2_eigen optik::optik)

pluginlib_export_plugin_description_file(moveit_core
                                         optik_kinematics_description.xml)

install(DIRECTORY include/ DESTINATION include/optik_kinematics_plugin)
install(
  TARGETS optik_kinematics_plugin
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core pluginlib rclcpp tf2_geometry_msgs
                          tf2_eigen optik)
ament_package()
