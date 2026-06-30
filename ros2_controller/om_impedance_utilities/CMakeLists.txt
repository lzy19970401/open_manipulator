cmake_minimum_required(VERSION 3.16)
project(om_impedance_utilities LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_gtest(test_impedance_utilities test/test_impedance_utilities.cpp)
  if(TARGET test_impedance_utilities)
    target_compile_features(test_impedance_utilities PUBLIC cxx_std_17)
    target_link_libraries(test_impedance_utilities ${PROJECT_NAME})
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)
ament_package()
