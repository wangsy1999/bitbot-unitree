cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(CtrlZ)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(UNIX)
    set(CMAKE_THREAD_LIBS_INIT "-lpthread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)

find_package(nlohmann_json REQUIRED)
#aux_source_directory(Schedulers SRC_SCHEDULERS)
#aux_source_directory(Workers SRC_WORKERS)
#aux_source_directory(Utils SRC_UTILS)

file(GLOB_RECURSE SRC_SCHEDULERS Schedulers/*.cpp Schedulers/*.h)
file(GLOB_RECURSE SRC_WORKERS Workers/*.cpp Workers/*.h)
file(GLOB_RECURSE SRC_UTILS Utils/*.cpp Utils/*.h)


message("
Ctrl-Z is a muti-thread RL deployment framework for Robot locomotion.")
message("Visit http://opensource.zzshub.cn/CtrlZ for detailed documentation.")

message("+----------------------------------------+
|  CCCC  TTTTT  RRRRR  L           ZZZZZ |
| C        T    R   R  L              Z  |
| C        T    RRRRR  L    =====    Z   |
| C        T    R R    L            Z    |
|  CCCC    T    R  RR  LLLLL       ZZZZZ |
+----------------------------------------+
")

# include directory for all target
add_library(CtrlZ STATIC
    ${SRC_SCHEDULERS}
    ${SRC_WORKERS}
    ${SRC_UTILS}
)

add_subdirectory(Extensions)

target_include_directories(CtrlZ PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Resolve the native ONNX Runtime library (.dylib on macOS, .so on Linux,
# import library on Windows). CPU distributions need no separate provider lib.
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS "${ONNXRUNTIME_ROOT}"
    PATH_SUFFIXES include include/onnxruntime include/onnxruntime/core/session
    REQUIRED)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
    HINTS "${ONNXRUNTIME_ROOT}"
    PATH_SUFFIXES lib lib64
    REQUIRED)
add_library(CtrlZOnnxRuntime UNKNOWN IMPORTED)
set_target_properties(CtrlZOnnxRuntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")
target_link_libraries(CtrlZ PUBLIC CtrlZOnnxRuntime)

#set(READERWRITERQUEUE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)
#target_include_directories(CtrlZ PUBLIC ${READERWRITERQUEUE_INCLUDE_DIRS})
target_link_libraries(CtrlZ PUBLIC
    nlohmann_json::nlohmann_json
)
