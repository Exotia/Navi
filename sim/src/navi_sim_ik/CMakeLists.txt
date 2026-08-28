cmake_minimum_required(VERSION 3.8)
project(navi_sim_ik)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Simulink-generated code, compiled as found. Its warnings are not ours to
# fix and would drown anything we do need to see.
file(GLOB VENDOR_SOURCES vendor/ert_rtw/*.cpp vendor/IkController.cpp)
add_library(navi_sim_ik_model STATIC ${VENDOR_SOURCES})
target_include_directories(navi_sim_ik_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/ert_rtw>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_options(navi_sim_ik_model PRIVATE -w)

add_library(navi_sim_ik_stepper STATIC src/sim_ik_stepper.cpp)
target_link_libraries(navi_sim_ik_stepper navi_sim_ik_model)
target_include_directories(navi_sim_ik_stepper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_executable(sim_ik_node src/sim_ik_node.cpp)
target_link_libraries(sim_ik_node navi_sim_ik_stepper)
ament_target_dependencies(sim_ik_node
  rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs trajectory_msgs)
install(TARGETS sim_ik_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vendored_model test/test_vendored_model.cpp)
  target_link_libraries(test_vendored_model navi_sim_ik_model)

  ament_add_gtest(test_sim_ik_stepper test/test_sim_ik_stepper.cpp)
  target_link_libraries(test_sim_ik_stepper navi_sim_ik_stepper)
endif()

ament_package()
