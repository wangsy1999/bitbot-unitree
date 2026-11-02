cmake_minimum_required(VERSION 3.14)

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
option(BUILD_REALSENSE "Build the hardware RealSense depth worker" ON)

if(BUILD_SIMULATION)
  add_definitions(-DBUILD_SIMULATION)
  BitbotAddDependency(
    NAME bitbot_mujoco
    FILE_NAME "bitbot_mujoco-main.zip"
    FILE_PATH ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
    USE_LOCAL_FILE TRUE
  )
else()
  add_subdirectory(Bitbot_Unitree)
  BitbotAddDependency(
    NAME bitbot_kernel
    FILE_NAME "bitbot_kernel-main.zip"
    FILE_PATH ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
    USE_LOCAL_FILE TRUE
  )
endif()


set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DPROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}")
message("PROJECT_ROOT_DIR: ${PROJECT_ROOT_DIR}")

add_executable(main_app
  main.cpp
  types.hpp
  user_func.h
  user_func.cpp
  user_command.h
  user_command.cpp
)

if(BUILD_SIMULATION)
  target_link_libraries(main_app BitbotMujoco)
else()
  target_link_libraries(main_app bitbotUnitree)
endif()

target_link_libraries(main_app
  BitbotKernel
  CtrlZ
)

target_compile_options(main_app PUBLIC)
target_compile_features(main_app PUBLIC cxx_std_20)

if(BUILD_SIMULATION)
  find_package(OpenGL REQUIRED)
  target_sources(main_app PRIVATE
    parkour/depth_camera.cpp
    parkour/mujoco_depth_camera.cpp
  )
  target_link_libraries(main_app OpenGL::GL)
endif()

# RealSense is a hardware-only camera source; simulation retains MuJoCo depth.
if(NOT BUILD_SIMULATION AND BUILD_REALSENSE)
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
