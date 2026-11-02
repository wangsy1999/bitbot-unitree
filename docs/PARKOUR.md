# Parkour 相机与推理接入

Parkour 使用原来的 `main_app`、`types.hpp`、`user_func` 和 CtrlZ 调度器。
`main.cpp`、`user_command.h/.cpp` 未修改，复位、行走、跳舞切换沿用原有事件和状态机。
不再提供独立的 `parkour_sim` 入口或专用按键配置。

## 编译与运行

```bash
cmake -S . -B build -DBUILD_SIMULATION=ON -DONNXRUNTIME_ROOT=/path/to/onnxruntime
cmake --build build --target main_app -j4
cd build/bin
./main_app
```

沿用原项目的 Bitbot Copilot 操作顺序 `9 → 8 → p → r`，复位完成后再按 `r`。
速度、手柄和跳舞按键继续使用 `settings/setting.json`。终端不读取这些按键。
原有事件日志中的 `policy run (DreamWaQ)` 文本未修改，实际推理类型由配置决定。

`settings/CtrlConfig.json` 中 `Workers.WalkNet1` 增加：

```json
{
  "Policy": "Parkour",
  "ParkourModels": "CheckPoints/parkour",
  "ParkourConfig": "settings/g1_parkour.json"
}
```

MuJoCo 构建支持 `Policy: "DreamWaQ"` 使用原行走推理实现。切回原 DreamWaQ
仿真参数时，需同时将 `settings/bitbot_mujoco.xml` 的场景恢复为
`../models/scene_29dof.xml`，并恢复原设备 PD 参数。
实机编译仍使用原 DreamWaQ 类型，本次不接入实机相机。

## 接入位置

- `types.hpp`：声明完整 Parkour 观测快照和行走推理 Worker 类型，继续使用
  `WalkNet1Action`、`WalkNet1NetLastAction` 等现有输出。
- `user_func.h/.cpp`：通过 `bus.GetDevice()` 获取相机，设置 `SetSubsampleSteps()`，
  在 MainTask 注册观测回调 Worker，配置 Parkour 的复位角及 PD。
- `parkour/mujoco_depth_camera.h/.cpp`：注册 `MujocoDepthCamera` 设备，
  在总线 `Input()` 中按调用次数分频采集；`GetDepth()` 提供原始 OpenGL 深度缓冲。
  XML 中设备 ID 为 31，分辨率为 64×36，默认分频 10。
- `parkour/depth_camera_worker.hpp`：`DepthObservationPublisher` 由
  `SimpleCallbackWorker` 调用，对应参考项目的 `PublishDepthImageFlat` 回调职责。
  只读取新帧，完成米制转换、预处理和深度历史拼接，再发布完整观测快照。
- `parkour/inference_worker.hpp`：原 `InferWalkTask` 执行双网络推理，目标按
  `JOINT_ID_MAP` 转为原 scheduler 顺序，继续交给原 ActionManager 和 MotorWorker。
- `parkour/policy.hpp`：双网络 ONNX、本体观测历史和公共深度预处理函数。
  深度图像历史由观测层维护。`reset()` 通过 generation 分别清空本体和图像历史，
  并丢弃复位期间的在途结果。
- `parkour/depth_camera.*`：MuJoCo 离屏深度采集。相机由 UI 线程创建/销毁，
  渲染仅在物理回调执行，推理线程通过 scheduler 接收完整快照，不访问实时 mjData。

调度周期保持原设置：物理控制 2ms，`InferWalkTask` 分频 10，对应 50Hz。
相机采集按设备 `Input()` 次数分频，不以 MuJoCo 时间作为采集门限；
没有仿真时间回退检测，也不接管 MuJoCo Reset 按钮。
Parkour 的默认角、kp/kd 从源参数转换到原 scheduler 关节顺序。
原来的跳舞模型、网络混合、状态切换和命令 Worker 保持原实现。

## 相机与策略参数

相机采集参考
`/home/wsy/Documents/vision/CtrlZ/example/BitbotSimulation/dependencies/bitbot_mujoco-src/`
里的 `MujocoDepthCamera`，使用离屏缓冲和 `mjr_readPixels`，无 RGB 读回。
预览窗口显示放大的 32×18 策略深度图，近处白、远处黑；关闭预览后采集继续运行。
代码已放入本项目，不依赖上述本地目录，也不修改依赖压缩包。

相机安装在 `torso_link`，位置 `(0.0487988662, 0.01, 0.4378029938)` m，
位姿来自源项目 MuJoCo adapter，已从 forward/left/up 转换为 OpenGL right/up/back。
分辨率 64×36、垂直 FOV 58.29°、水平 FOV 约 89.5°。
仅渲染碰撞 group 0，排除视觉副本与 site。

针对项目自带的 **MuJoCo 2.3.7**，将非线性 OpenGL 深度通过模型 near/far 和
extent 转换为光轴 Z 米制距离，再上下翻转为首行在顶部。不乘射线长度，MSAA 关闭，
未命中像素作为无穷远。升级渲染版本时需要核对深度缓冲约定。

保持源策略的仿真预处理：取行 `[18:36]`、列 `[16:48]`，3×3 Gaussian
（sigma=1，reflect 边界），截断到 [0,2.5]m 后除以 2.5。
37 帧图像缓存每隔 5 帧抽取一次，共 8 帧，从旧到新；延迟由
`depth_delay_frames` 决定。首帧重复填满图像和本体观测历史。

- 深度编码器：float32 `[1,8,18,32] → [1,128]`。
- 本体观测：torso 角速度×0.25、重力、命令、相对关节角、关节速度×0.05、上一动作；
  8 帧按观测项排列为 768 维，拼接深度 latent。
- Actor：float32 `[1,896] → [1,29]`。
- 关节输入先按名称重排并应用 `joint_sign`，目标经同一符号转回设备方向。

`models/g1_parkour.xml` 从原 G1 模型增加相机，并按源策略设置 armature、
移除被动 damping/frictionloss，actuator 范围对应源 effort。
`models/scene_parkour.xml` 沿用原楼梯和网格。原 XML 文件保留。

## 来源与验证

模型和策略来自 `/home/wsy/rl/g1_deploy`：

- `models/parkour/0-depth_encoder.onnx` 与 `actor.onnx` 原样复制。
- `config/g1_parkour.json` 原样复制至 `settings/g1_parkour.json`。
- 推理逻辑改编自 `include/g1_parkour_policy.h`，保留仿真分支，配置读取改为
  nlohmann/json，移除实机 RealSense、OpenCV 和 UDP 接口。
- 相机位姿来自 `/home/wsy/rl/unitree_mujoco/example/parkour/mujoco_adapter.py`。

源项目 BSD 3-Clause 许可保留在 `parkour/LICENSE`。

已做双网络离线推理、MuJoCo 2.3.7 场景 XML 加载及 `main_app` 编译检查。
代码风格整理阶段做过一次含历史推进和复位的离线对照，关节目标与当时整理前完全一致。
本次改为设备采集和观测回调结构后，仅重新编译 `main_app`。
未运行图形仿真或 Isaac Sim，尚未验证相机实际画面及行走、爬楼效果。
