cmake_minimum_required(VERSION 3.16)
project(stand_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

# Task deployment assets: keep each model and its matching parameters together.
get_filename_component(G1_EXPORTED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../exported" ABSOLUTE)
set(G1_LOCOMOTION_DIR "${G1_EXPORTED_DIR}/g1_locomotion")
set(G1_PARKOUR_DIR "${G1_EXPORTED_DIR}/g1_parkour")

add_executable(stand_go2 stand_go2.cpp)
target_link_libraries(stand_go2  unitree_sdk2)

add_executable(gamepad_monitor gamepad_monitor.cpp)
target_compile_features(gamepad_monitor PRIVATE cxx_std_17)
target_include_directories(gamepad_monitor PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../include")
target_link_libraries(gamepad_monitor yaml-cpp)
target_compile_definitions(gamepad_monitor PRIVATE
    G1_GAMEPAD_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/gamepad.yaml")

SET(CMAKE_BUILD_TYPE Release)

option(BUILD_G1_LOCOMOTION "Build G1 ONNX locomotion controller" OFF)
option(BUILD_G1_PARKOUR "Build G1 parkour ONNX controller for synchronized MuJoCo observations" OFF)
option(BUILD_G1_CA_PE_AMP_VAE "Build G1 visual CA PE AMP VAE controller" OFF)
if(BUILD_G1_LOCOMOTION OR BUILD_G1_PARKOUR OR BUILD_G1_CA_PE_AMP_VAE)
    set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime C/C++ distribution directory")
    find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
        HINTS "${ONNXRUNTIME_ROOT}/include")
    find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime libonnxruntime.so.1
        HINTS "${ONNXRUNTIME_ROOT}/lib")
    if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
        message(FATAL_ERROR "Set ONNXRUNTIME_ROOT to an ONNX Runtime C/C++ distribution (include/ and lib/)")
    endif()
endif()
if(BUILD_G1_PARKOUR)
    add_executable(g1_parkour g1_parkour.cpp)
    target_compile_features(g1_parkour PRIVATE cxx_std_17)
    target_include_directories(g1_parkour PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../../include")
    target_link_libraries(g1_parkour unitree_sdk2 yaml-cpp "${ONNXRUNTIME_LIBRARY}")
    target_compile_definitions(g1_parkour PRIVATE
        G1_PARKOUR_DEFAULT_CONFIG="${G1_PARKOUR_DIR}/g1_torsobase_cfg.json"
        G1_PARKOUR_DEFAULT_MODELS="${G1_PARKOUR_DIR}"
        G1_GAMEPAD_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/gamepad.yaml")
    if(UNIX AND NOT APPLE)
        target_link_options(g1_parkour PRIVATE "-Wl,--disable-new-dtags")
    endif()
endif()
if(BUILD_G1_LOCOMOTION)
    add_executable(g1_locomotion g1_locomotion.cpp)
    target_compile_features(g1_locomotion PRIVATE cxx_std_17)
    target_include_directories(g1_locomotion PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../../include")
    target_link_libraries(g1_locomotion unitree_sdk2 Boost::boost yaml-cpp "${ONNXRUNTIME_LIBRARY}")
    if(UNIX AND NOT APPLE)
        # Use the linked SDK/ORT versions even in a shell that has sourced ROS.
        target_link_options(g1_locomotion PRIVATE "-Wl,--disable-new-dtags")
    endif()
    target_compile_definitions(g1_locomotion PRIVATE
        G1_POLICY_DEFAULT_CONFIG="${G1_LOCOMOTION_DIR}/g1_locomotion.yaml"
        G1_POLICY_DEFAULT_MODEL="${G1_LOCOMOTION_DIR}/policy.onnx"
        G1_GAMEPAD_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/gamepad.yaml")
endif()

if(BUILD_G1_CA_PE_AMP_VAE)
    add_executable(g1_ca_pe_amp_vae g1_ca_pe_amp_vae.cpp)
    target_compile_features(g1_ca_pe_amp_vae PRIVATE cxx_std_17)
    target_include_directories(g1_ca_pe_amp_vae PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../../include")
    target_link_libraries(g1_ca_pe_amp_vae unitree_sdk2 yaml-cpp "${ONNXRUNTIME_LIBRARY}")
    target_compile_definitions(g1_ca_pe_amp_vae PRIVATE
        G1_CA_PE_DEFAULT_CONFIG="${G1_EXPORTED_DIR}/g1_CA_PE_AMP_VAE/g1_ca_pe_amp_vae_cfg.json"
        G1_CA_PE_DEFAULT_MODELS="${G1_EXPORTED_DIR}/g1_CA_PE_AMP_VAE"
        G1_GAMEPAD_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../../simulate/gamepad.yaml")
    if(UNIX AND NOT APPLE)
        target_link_options(g1_ca_pe_amp_vae PRIVATE "-Wl,--disable-new-dtags")
    endif()
endif()
