################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.16)
project(om_pinocchio_gravity_compensation_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  pinocchio
  kdl_parser
  orocos_kdl
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
generate_parameter_library(pinocchio_gravity_compensation_controller_parameters
  src/pinocchio_gravity_compensation_controller_parameters.yaml
)

add_library(pinocchio_gravity_compensation_controller SHARED
  src/pinocchio_gravity_compensation_controller.cpp
)
target_compile_features(pinocchio_gravity_compensation_controller PUBLIC cxx_std_17)
target_include_directories(pinocchio_gravity_compensation_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/pinocchio_gravity_compensation_controller>
)
target_link_libraries(pinocchio_gravity_compensation_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pinocchio::pinocchio
  ${backward_ros_LIBRARIES}
  pinocchio_gravity_compensation_controller_parameters
)

target_compile_definitions(pinocchio_gravity_compensation_controller PRIVATE
  "PINOCCHIO_GRAVITY_COMPENSATION_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface pinocchio_gravity_compensation_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_index_cpp REQUIRED)
  find_package(open_manipulator_description REQUIRED)

  ament_add_gtest(test_gravity_parity test/test_gravity_parity.cpp)
  if(TARGET test_gravity_parity)
    target_compile_features(test_gravity_parity PUBLIC cxx_std_17)
    target_link_libraries(test_gravity_parity
      pinocchio::pinocchio
      ${kdl_parser_TARGETS}
      ${orocos_kdl_TARGETS}
      ament_index_cpp::ament_index_cpp
    )
  endif()

  ament_add_gtest(test_friction_compensation test/test_friction_compensation.cpp)
  if(TARGET test_friction_compensation)
    target_compile_features(test_friction_compensation PUBLIC cxx_std_17)
  endif()
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include/pinocchio_gravity_compensation_controller
)
install(TARGETS
    pinocchio_gravity_compensation_controller
    pinocchio_gravity_compensation_controller_parameters
  EXPORT export_pinocchio_gravity_compensation_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_pinocchio_gravity_compensation_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
