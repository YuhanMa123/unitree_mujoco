cmake_minimum_required(VERSION 3.16)
project(stand_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(stand_go2 stand_go2.cpp)
target_link_libraries(stand_go2  unitree_sdk2)

add_executable(reset_g1_29dof reset_g1_29dof.cpp)
target_compile_features(reset_g1_29dof PRIVATE cxx_std_17)
target_link_libraries(reset_g1_29dof unitree_sdk2 Boost::boost yaml-cpp)
target_compile_definitions(reset_g1_29dof PRIVATE
    G1_POLICY_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/g1_locomotion.yaml")

SET(CMAKE_BUILD_TYPE Release)

option(BUILD_G1_LOCOMOTION "Build G1 ONNX locomotion controller" OFF)
if(BUILD_G1_LOCOMOTION)
    set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime C/C++ distribution directory")
    find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
        HINTS "${ONNXRUNTIME_ROOT}/include")
    find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime libonnxruntime.so.1
        HINTS "${ONNXRUNTIME_ROOT}/lib")
    if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
        message(FATAL_ERROR "Set ONNXRUNTIME_ROOT to an ONNX Runtime C/C++ distribution (include/ and lib/)")
    endif()
    add_executable(g1_locomotion reset_g1_29dof.cpp)
    target_compile_features(g1_locomotion PRIVATE cxx_std_17)
    target_include_directories(g1_locomotion PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}")
    target_link_libraries(g1_locomotion unitree_sdk2 Boost::boost yaml-cpp "${ONNXRUNTIME_LIBRARY}")
    if(UNIX AND NOT APPLE)
        # Use the linked SDK/ORT versions even in a shell that has sourced ROS.
        target_link_options(g1_locomotion PRIVATE "-Wl,--disable-new-dtags")
    endif()
    target_compile_definitions(g1_locomotion PRIVATE G1_WITH_POLICY
        G1_POLICY_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/g1_locomotion.yaml"
        G1_POLICY_DEFAULT_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../../policy.onnx")
    include(CTest)
    if(BUILD_TESTING)
        add_executable(g1_hardware_test tests/g1_hardware_test.cpp)
        target_compile_features(g1_hardware_test PRIVATE cxx_std_17)
        target_include_directories(g1_hardware_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
        target_link_libraries(g1_hardware_test unitree_sdk2 yaml-cpp)
        if(UNIX AND NOT APPLE)
            target_link_options(g1_hardware_test PRIVATE "-Wl,--disable-new-dtags")
        endif()
        add_test(NAME g1_hardware COMMAND g1_hardware_test "${CMAKE_CURRENT_SOURCE_DIR}/g1_locomotion.yaml")
        add_executable(g1_locomotion_policy_test tests/g1_locomotion_policy_test.cpp)
        target_compile_features(g1_locomotion_policy_test PRIVATE cxx_std_17)
        target_include_directories(g1_locomotion_policy_test PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}" "${ONNXRUNTIME_INCLUDE_DIR}")
        target_link_libraries(g1_locomotion_policy_test yaml-cpp "${ONNXRUNTIME_LIBRARY}")
        if(UNIX AND NOT APPLE)
            target_link_options(g1_locomotion_policy_test PRIVATE "-Wl,--disable-new-dtags")
        endif()
        add_test(NAME g1_locomotion_policy COMMAND g1_locomotion_policy_test
            "${CMAKE_CURRENT_SOURCE_DIR}/../../policy.onnx"
            "${CMAKE_CURRENT_SOURCE_DIR}/g1_locomotion.yaml")
    endif()
endif()
