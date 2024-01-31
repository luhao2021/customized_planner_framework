cmake_minimum_required(VERSION 3.8)
project(planner_library_hao)

set(PLANNER_LIB_NAME planner_library_hao)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED)

include_directories(include)
include_directories(/opt/ros/humble/include/ompl-1.6)

add_library(${PLANNER_LIB_NAME} SHARED
  src/MyRRT.cpp
)
ament_target_dependencies(${PLANNER_LIB_NAME}
  ompl
)
target_link_libraries(${PLANNER_LIB_NAME}
	-lboost_graph
)

install(
  TARGETS ${PLANNER_LIB_NAME}
  EXPORT ${PLANNER_LIB_NAME}Targets
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PLANNER_LIB_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
