cmake_minimum_required(VERSION 3.5)
project(hungarian_solver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(
  hungarian_solver
  src/hungarian_solver.cpp
)
ament_target_dependencies(hungarian_solver rclcpp)

# install executables/libs
install(TARGETS
  hungarian_solver
  DESTINATION lib/hungarian_solver
)

# Install header files
install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_test_hungarian_solver test/src/test_hungarian_solver.cpp)
  target_link_libraries(test_test_hungarian_solver hungarian_solver)
endif()

ament_export_include_directories(include)
ament_package()