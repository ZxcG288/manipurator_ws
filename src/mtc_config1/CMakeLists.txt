cmake_minimum_required(VERSION 3.8)
project(mtc_config1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)

# Function to reduce duplicate code
function(add_mtc_executable target_name source_file)
  add_executable(${target_name} src/${source_file})
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(${target_name}
    "rclcpp"
    "geometry_msgs"
  )
endfunction()

# Add executables
add_mtc_executable(mtc_node mtc_node.cpp)
add_mtc_executable(mtc_node_test mtc_node_test.cpp)
add_mtc_executable(mtc_pub mtc_pub.cpp)
add_mtc_executable(start_mtc start_mtc.cpp)
add_mtc_executable(place_node place_node.cpp)
add_mtc_executable(moveit_controller moveit_controller.cpp)
add_mtc_executable(mtc_place mtc_place.cpp)
add_mtc_executable(place_pub place_pub.cpp)
add_mtc_executable(mtc_ready_to_go mtc_ready_to_go.cpp)

# Additional dependencies for MoveIt Task Constructor nodes
foreach(target mtc_node mtc_node_test start_mtc place_node moveit_controller mtc_place place_pub mtc_ready_to_go)
  ament_target_dependencies(${target} "moveit_task_constructor_core")
endforeach()

# Install targets
install(TARGETS
  mtc_node mtc_node_test mtc_pub start_mtc place_node moveit_controller mtc_place place_pub mtc_ready_to_go 
  DESTINATION lib/${PROJECT_NAME})

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
