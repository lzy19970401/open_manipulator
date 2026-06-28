################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.16)
project(om_pinocchio_impedance_controller LANGUAGES CXX)

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
generate_parameter_library(pinocchio_impedance_controller_parameters
  src/pinocchio_impedance_controller_parameters.yaml
)

add_library(pinocchio_impedance_controller SHARED
  src/pinocchio_impedance_controller.cpp
)
target_compile_features(pinocchio_impedance_controller PUBLIC cxx_std_17)
target_include_directories(pinocchio_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/pinocchio_impedance_controller>
)
target_link_libraries(pinocchio_impedance_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pinocchio::pinocchio
  ${backward_ros_LIBRARIES}
  pinocchio_impedance_controller_parameters
)

target_compile_definitions(pinocchio_impedance_controller PRIVATE
  "PINOCCHIO_IMPEDANCE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface pinocchio_impedance_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_index_cpp REQUIRED)
  find_package(open_manipulator_description REQUIRED)

  ament_add_gtest(test_impedance_torque test/test_impedance_torque.cpp)
  if(TARGET test_impedance_torque)
    target_compile_features(test_impedance_torque PUBLIC cxx_std_17)
    target_include_directories(test_impedance_torque PRIVATE
      ${PROJECT_SOURCE_DIR}/include
    )
    target_link_libraries(test_impedance_torque
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
  DESTINATION include/pinocchio_impedance_controller
)
install(TARGETS
    pinocchio_impedance_controller
    pinocchio_impedance_controller_parameters
  EXPORT export_pinocchio_impedance_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_pinocchio_impedance_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
