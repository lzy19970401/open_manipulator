################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.16)
project(om_pinocchio_task_impedance_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  om_impedance_utilities
  pluginlib
  proxsuite
  rclcpp
  rclcpp_lifecycle
  pinocchio
  urdf
)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

################################################################################
# Build
################################################################################
generate_parameter_library(pinocchio_task_impedance_controller_parameters
  src/pinocchio_task_impedance_controller_parameters.yaml
)

add_library(task_impedance_control_law SHARED
  src/task_impedance_qp_solver.cpp
  src/task_impedance_control_law.cpp
)
target_compile_features(task_impedance_control_law PUBLIC cxx_std_17)
target_include_directories(task_impedance_control_law PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/pinocchio_task_impedance_controller>
)
target_link_libraries(task_impedance_control_law PUBLIC
  om_impedance_utilities::om_impedance_utilities
  proxsuite::proxsuite
  pinocchio::pinocchio
)

add_library(pinocchio_task_impedance_controller SHARED
  src/pinocchio_task_impedance_controller.cpp
)
target_compile_features(pinocchio_task_impedance_controller PUBLIC cxx_std_17)
target_include_directories(pinocchio_task_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/pinocchio_task_impedance_controller>
)
target_link_libraries(pinocchio_task_impedance_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pinocchio::pinocchio
  ${backward_ros_LIBRARIES}
  pinocchio_task_impedance_controller_parameters
  task_impedance_control_law
)

target_compile_definitions(pinocchio_task_impedance_controller PRIVATE
  "PINOCCHIO_TASK_IMPEDANCE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface pinocchio_task_impedance_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_index_cpp REQUIRED)
  find_package(open_manipulator_description REQUIRED)

  ament_add_gtest(test_task_impedance_control_law test/test_task_impedance_control_law.cpp)
  if(TARGET test_task_impedance_control_law)
    target_compile_features(test_task_impedance_control_law PUBLIC cxx_std_17)
    target_include_directories(test_task_impedance_control_law PRIVATE
      ${PROJECT_SOURCE_DIR}/include
    )
    target_link_libraries(test_task_impedance_control_law
      task_impedance_control_law
      pinocchio::pinocchio
      ament_index_cpp::ament_index_cpp
    )
  endif()
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include/pinocchio_task_impedance_controller
)
install(TARGETS
    task_impedance_control_law
    pinocchio_task_impedance_controller
    pinocchio_task_impedance_controller_parameters
  EXPORT export_pinocchio_task_impedance_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_pinocchio_task_impedance_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
