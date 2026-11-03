cmake_minimum_required(VERSION 3.24)

project(bitbot)

set(BITBOT_DEPENDENCY_USE_PROXY OFF)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/dependencies)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


add_subdirectory(CtrlZ)
include(bitbot.cmake)

option(BUILD_SIMULATION "Build simulation" ON)
if(APPLE AND NOT BUILD_SIMULATION)
  message(FATAL_ERROR "macOS supports MuJoCo simulation only; set BUILD_SIMULATION=ON")
endif()
option(BUILD_REALSENSE "Build the hardware RealSense depth worker" ON)
option(BUILD_CAMERA_APP "Build the camera/Parkour application" ON)
option(BUILD_DWAQ_APP "Build the standalone DreamWaQ application" ON)

if(BUILD_SIMULATION)
  add_definitions(-DBUILD_SIMULATION)
  option(BITBOT_MUJOCO_BUILD_TESTS "Build headless MuJoCo compatibility checks" OFF)
  if(BITBOT_MUJOCO_BUILD_TESTS)
    enable_testing()
  endif()
  add_subdirectory(Bitbot_Mujoco)
else()
  add_subdirectory(Bitbot_Unitree)
  BitbotAddDependency(
    NAME bitbot_kernel
    FILE_NAME "bitbot_kernel-main.zip"
    FILE_PATH ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
    USE_LOCAL_FILE TRUE
  )
endif()


if(APPLE)
  # fmt 10.1.1's consteval parser is incompatible with recent AppleClang.
  target_compile_definitions(fmt PUBLIC FMT_CONSTEVAL=)
  target_compile_definitions(fmt-header-only INTERFACE FMT_CONSTEVAL=)
  include(cmake/macos_compat.cmake)
  target_sources(BitbotKernel PRIVATE cmake/macos_time.c)
endif()

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DPROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}")
message("PROJECT_ROOT_DIR: ${PROJECT_ROOT_DIR}")

# Each application owns its main/types/user callbacks; SDK and CtrlZ remain common dependencies.
function(BitbotAddApplication target source_directory)
  add_executable(${target}
    "${source_directory}/main.cpp"
    "${source_directory}/types.hpp"
    "${source_directory}/user_func.h"
    "${source_directory}/user_func.cpp"
    "${source_directory}/user_command.h"
    "${source_directory}/user_command.cpp"
  )
  target_include_directories(${target} PRIVATE "${PROJECT_SOURCE_DIR}")
  if(BUILD_SIMULATION)
    target_link_libraries(${target} BitbotMujoco)
  else()
    target_link_libraries(${target} bitbotUnitree)
  endif()
  target_link_libraries(${target} BitbotKernel CtrlZ)
  target_compile_features(${target} PUBLIC cxx_std_20)
endfunction()

if(BUILD_CAMERA_APP)
  BitbotAddApplication(main_app "${PROJECT_SOURCE_DIR}")
  if(BUILD_SIMULATION)
    find_package(OpenGL REQUIRED)
    target_sources(main_app PRIVATE
      parkour/depth_camera.cpp
      parkour/mujoco_depth_camera.cpp
    )
    target_link_libraries(main_app OpenGL::GL)
  elseif(BUILD_REALSENSE)
    find_package(realsense2 CONFIG REQUIRED)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
    target_compile_definitions(main_app PRIVATE BUILD_REALSENSE)
    target_include_directories(main_app PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(main_app ${OpenCV_LIBS})
    if(TARGET realsense2::realsense2)
      target_link_libraries(main_app realsense2::realsense2)
    elseif(TARGET realsense2)
      target_link_libraries(main_app realsense2)
    else()
      target_include_directories(main_app PRIVATE ${realsense2_INCLUDE_DIR})
      target_link_libraries(main_app ${realsense2_LIBRARY})
    endif()
  endif()
endif()

if(BUILD_DWAQ_APP)
  BitbotAddApplication(dwaq_app "${PROJECT_SOURCE_DIR}/src_dwaq")
  target_sources(dwaq_app PRIVATE src_dwaq/app_config.hpp)
  target_compile_definitions(dwaq_app PRIVATE
    BITBOT_CTRL_CONFIG="src_dwaq/settings/CtrlConfig.json"
    BITBOT_MUJOCO_CONFIG="src_dwaq/settings/bitbot_mujoco.xml"
    BITBOT_UNITREE_CONFIG="src_dwaq/settings/bitbot_unitree.xml"
  )
endif()
