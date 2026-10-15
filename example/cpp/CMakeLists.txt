cmake_minimum_required(VERSION 3.16)
project(stand_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED)

add_executable(stand_go2 stand_go2.cpp)
target_link_libraries(stand_go2  unitree_sdk2)

add_executable(reset_g1_29dof reset_g1_29dof.cpp)
target_compile_features(reset_g1_29dof PRIVATE cxx_std_17)
target_link_libraries(reset_g1_29dof unitree_sdk2 Boost::boost)
target_compile_definitions(reset_g1_29dof PRIVATE
    G1_RESET_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/g1_reset.xml")

SET(CMAKE_BUILD_TYPE Release)
