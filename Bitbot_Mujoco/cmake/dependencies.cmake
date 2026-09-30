include(${CMAKE_CURRENT_LIST_DIR}/../../bitbot.cmake)

if(NOT TARGET BitbotKernel)
  BitbotAddDependency(
    NAME bitbot_kernel
    FILE_NAME "bitbot_kernel-main.zip"
    FILE_PATH ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
    USE_LOCAL_FILE TRUE
  )
endif()

# Pin the engine and its simulate headers together. A system engine alone does
# not provide the viewer source interfaces used by Bitbot.
set(BITBOT_MUJOCO_VERSION "3.14.0")
set(BITBOT_MUJOCO_ARCHIVE "" CACHE FILEPATH "Optional local MuJoCo 3.14.0 source archive")
set(_mujoco_url "https://github.com/google-deepmind/mujoco/archive/refs/tags/${BITBOT_MUJOCO_VERSION}.tar.gz")
if(BITBOT_MUJOCO_ARCHIVE)
  set(_mujoco_url "${BITBOT_MUJOCO_ARCHIVE}")
elseif(EXISTS "${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}/mujoco-${BITBOT_MUJOCO_VERSION}.tar.gz")
  set(_mujoco_url "${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}/mujoco-${BITBOT_MUJOCO_VERSION}.tar.gz")
endif()

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE ON CACHE BOOL "" FORCE)
set(SIMULATE_BUILD_EXECUTABLE OFF CACHE BOOL "" FORCE)

FetchContent_Declare(mujoco
  URL "${_mujoco_url}"
  URL_HASH SHA256=4e1ebd14406967f4554a9c6ed613fe7f88a8ea8a9c272ede80d083a25f9e31a9
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(mujoco)

if(NOT TARGET mujoco::libmujoco_simulate)
  message(FATAL_ERROR "Bitbot requires the MuJoCo 3.14.0 simulate library")
endif()
