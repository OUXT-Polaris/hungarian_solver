cmake_minimum_required(VERSION 3.5)
project(hungarian_solver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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

#if (CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  find_package(code_coverage REQUIRED)
#  if(ENABLE_COVERAGE_TESTING)
#    include(CodeCoverage)
#    APPEND_COVERAGE_COMPILER_FLAGS()
#  endif()

#  add_rostest_gtest(test-hungarian_solver
#  test/test_hungarian_solver.test
#  test/src/test_hungarian_solver.cpp)
#  target_link_libraries(test-hungarian_solver
#  ${catkin_LIBRARIES}
#  hungarian_solver)

#  if(ENABLE_COVERAGE_TESTING)
#    set(COVERAGE_EXCLUDES "*/hungarian_solver/test*" "*/hungarian_solver/coverage")
#    add_code_coverage(
#      NAME hungarian_solver_coverage
#      DEPENDS tests
#    )
#  endif()
#endif ()

ament_export_include_directories(include)
ament_package()