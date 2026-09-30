cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(bitbot_mujoco)

include(cmake/dependencies.cmake)

set(BITBOT_MUJOCO_INCLUDED TRUE)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/device BITBOT_MUJOCO_DEVICE_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/bus BITBOT_MUJOCO_BUS_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/kernel BITBOT_MUJOCO_KERNEL_SRC)

set(BITBOT_MUJOCO_SRC
  ${BITBOT_MUJOCO_DEVICE_SRC}
  ${BITBOT_MUJOCO_BUS_SRC}
  ${BITBOT_MUJOCO_KERNEL_SRC}
)

add_library(BitbotMujoco STATIC ${BITBOT_MUJOCO_SRC})
target_include_directories(BitbotMujoco PUBLIC
  $<BUILD_INTERFACE:${THIRD_PARTY_INCLUDE}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/bitbot_kernel/3rd_party/usocket>
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/bitbot_kernel/3rd_party/uwebsocket>
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/usockets>
  # $<BUILD_INTERFACE:${OPENSSL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(BitbotMujoco PRIVATE
                           $<$<PLATFORM_ID:Windows>:_USE_MATH_DEFINES>
)
target_link_libraries(BitbotMujoco PUBLIC
                      BitbotKernel
                      mujoco::mujoco
                      mujoco::libmujoco_simulate
                      glfw
)

# The simulate viewer uses CVDisplayLink through its platform adapter on macOS.
if(APPLE)
  target_link_libraries(BitbotMujoco PUBLIC "-framework CoreVideo")
endif()

if(BITBOT_MUJOCO_BUILD_TESTS)
  add_subdirectory(tests)
endif()
