cmake_minimum_required(VERSION 3.8)
project(navi_sim_bringup)
find_package(ament_cmake REQUIRED)

# The real URDF is copied in at build time rather than referenced from the
# working tree, so an installed package does not depend on the source
# checkout still being there.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../../../asterope_iiI.urdf
               ${CMAKE_CURRENT_BINARY_DIR}/asterope_base.urdf COPYONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/asterope_base.urdf
        DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY urdf worlds launch DESTINATION share/${PROJECT_NAME})

# sim_bridge is a script rather than a module: this package is ament_cmake
# (it exists for a world file, a xacro and a launch file), and adding an
# ament_python package for one node would mean two packages where the launch
# file and the node it launches could drift apart.
install(PROGRAMS scripts/sim_bridge.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_bridge test/test_sim_bridge.py TIMEOUT 120)
endif()

ament_package()
