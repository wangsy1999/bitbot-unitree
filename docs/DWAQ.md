# 独立 DWAQ 程序

在最新 camera 分支上保留两个可分别运行的程序：

| 程序 | 行走推理 | 控制配置 | 相机 |
| --- | --- | --- | --- |
| `main_app` | 当前 camera/Parkour 配置 | `settings/CtrlConfig.json` | MuJoCo 或真机 RealSense |
| `dwaq_app` | DreamWaQ | `src_dwaq/settings/CtrlConfig.json` | 不初始化相机，不需要 RealSense |

DWAQ 的入口和控制代码独立放在 `src_dwaq/`，不编译根目录 camera 程序的
`main.cpp`、`types.hpp`、`user_func` 或 `user_command`。目录结构：

```text
src_dwaq/
  main.cpp
  types.hpp
  user_func.h / user_func.cpp
  user_command.h / user_command.cpp
  app_config.hpp
  settings/
    CtrlConfig.json
    bitbot_mujoco.xml
    bitbot_unitree.xml
    setting.json
```

保留最新 camera 分支的设备驱动、SDK 和 CtrlZ 作为公共依赖，模型及网格继续使用根目录
`CheckPoints/`、`models/`。两个程序的源码和运行配置各自独立，状态及按键行为沿用原设计。
原 camera 源码和 JSON/XML 保留。程序沿用相同的设备与后端端口，应分别启动。

## MuJoCo 编译和启动

```bash
cmake -S . -B build -DBUILD_SIMULATION=ON \
  -DBUILD_CAMERA_APP=ON -DBUILD_DWAQ_APP=ON \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime
cmake --build build --target main_app dwaq_app -j2

# 按需要分别运行
./build/bin/main_app
./build/bin/dwaq_app
```

`dwaq_app` 使用 `src_dwaq/settings/bitbot_mujoco.xml`，加载原来的
`models/scene_29dof.xml` 和 G1 模型，不使用 Parkour 相机模型。
设备 PD 从 DWAQ 的 `ActionManager.MotorProperties` 按关节映射生成，确保进入策略前
复位阶段也使用 DWAQ 增益。

## 仅编译真机 DWAQ

```bash
cmake -S . -B build_dwaq_real -DBUILD_SIMULATION=OFF \
  -DBUILD_CAMERA_APP=OFF -DBUILD_DWAQ_APP=ON \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime
cmake --build build_dwaq_real --target dwaq_app -j2
```

关闭 `BUILD_CAMERA_APP` 后不会查找 RealSense 或 OpenCV，相机 SDK 不阻碍单独构建 DWAQ。
仍需要原项目的 Unitree SDK 和目标平台 C++ ONNX Runtime。
真机程序使用 `src_dwaq/settings/bitbot_unitree.xml`，网卡等参数在这份配置中调整。

## 模型和参数

复用已有 `CheckPoints/policy_dwaq.onnx` 和 `DreamWaQInferenceWorker`：

- 本体输入 float32 `[1,96]`，五帧历史 `[1,480]`，输出 `[1,29]`。
- pelvis IMU，本体历史按观测项排列；进入行走时清空上一动作和历史。
- 控制周期 2ms，策略周期 20ms。
- 默认姿态、PD、观测缩放、动作缩放及命令范围沿用当前项目已有的 DWAQ 适配，
  对应 `/home/wsy/rl/g1_deploy/config/g1_locomotion.yaml`。
- DWAQ 配置移除了相机相关章节；模型路径仍由 JSON 管理。
- 跳舞 Worker 和原有切换流程保留。

DWAQ 路径通过 `src_dwaq/app_config.hpp` 统一解析：程序配置文件相对编译时项目根目录，绝对路径原样使用。
JSON 中现有模型和轨迹绝对路径保持原设置；换机器部署时应确认这些路径存在。

通过 Bitbot Copilot 使用原 `9 → 8 → p → 等待复位 → r` 流程。
本次检查仅覆盖两个程序的 MuJoCo 构建和配置一致性，未运行仿真或真机验证。
