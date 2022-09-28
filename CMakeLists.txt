cmake_minimum_required(VERSION 3.5)
project(hiros_opensim_ik_solver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hiros_skeleton_msgs REQUIRED)

find_package(OpenSim REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${PROJECT_NAME}_node src/main.cpp
                                    src/IKSolver.cpp
                                    src/RTIKTool.cpp
                                    src/RTIMUPlacer.cpp
                                    src/Consumer.cpp
                                    src/Publisher.cpp
                                    src/utils.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs hiros_skeleton_msgs)

target_link_libraries(${PROJECT_NAME}_node ${OpenSim_LIBRARIES})

set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME
  ${PROJECT_NAME}
  PREFIX
  ""
)

target_include_directories(${PROJECT_NAME}_node
                           PRIVATE include
                           ${OpenSim_INCLUDE_DIRS})

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config models launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
