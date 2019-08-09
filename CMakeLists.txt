cmake_minimum_required(VERSION 2.8.3)
project(hungarian_solver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hungarian_solver
#  CATKIN_DEPENDS eigen roscpp
  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(
  hungarian_solver
  src/hungarian_solver.cpp
)
target_link_libraries(hungarian_solver ${catkin_LIBRARIES})

# install executables/libs
install(TARGETS hungarian_solver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(code_coverage REQUIRED)
  if(ENABLE_COVERAGE_TESTING)
    include(CodeCoverage)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  add_rostest_gtest(test-hungarian_solver
  test/test_hungarian_solver.test
  test/src/test_hungarian_solver.cpp)
  target_link_libraries(test-hungarian_solver
  ${catkin_LIBRARIES}
  hungarian_solver)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/hungarian_solver/test*" "*/hungarian_solver/coverage")
    add_code_coverage(
      NAME hungarian_solver_coverage
      DEPENDS tests
    )
  endif()
endif ()