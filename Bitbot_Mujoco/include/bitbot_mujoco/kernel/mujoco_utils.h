#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>
#include <functional>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <dirent.h>
  #include <dlfcn.h>
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}


std::string getExecutableDir();

// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries();

mjModel* LoadModel(const char* file, mujoco::Simulate& sim);
