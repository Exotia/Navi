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
ament_package()
