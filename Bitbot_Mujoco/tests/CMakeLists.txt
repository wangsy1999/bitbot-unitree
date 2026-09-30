add_executable(bitbot_mujoco_smoke_test mujoco_smoke_test.cpp)
target_link_libraries(bitbot_mujoco_smoke_test PRIVATE BitbotMujoco)
target_compile_features(bitbot_mujoco_smoke_test PRIVATE cxx_std_20)
add_test(NAME bitbot_mujoco_compatibility
  COMMAND bitbot_mujoco_smoke_test
    "${CMAKE_SOURCE_DIR}/models/scene_29dof.xml"
    "${CMAKE_SOURCE_DIR}/settings/bitbot_mujoco.xml")
