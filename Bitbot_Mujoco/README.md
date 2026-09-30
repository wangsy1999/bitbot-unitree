# Bitbot MuJoCo 3.14.0

此目录来自项目原有的 `dependencies/bitbot_mujoco-main.zip`，作为仓库内源码维护。
根项目的仿真分支直接构建此目录，旧 ZIP 和 MuJoCo 2.3.7 ZIP 不再参与该分支。
引擎和 `simulate` 窗口库一起固定到 MuJoCo 3.14.0，源码包有 SHA256 校验。

## 构建

需要 CMake 3.24+、C++20 编译器、ONNX Runtime C/C++ 开发包和平台窗口开发依赖。
首次配置会下载 MuJoCo 3.14.0 及其依赖。Linux 需要 GLFW 使用的 X11/Wayland
开发包；macOS 需要 Xcode Command Line Tools，并使用与编译目标匹配的 ARM64
或 x86_64 ONNX Runtime。CPU 版即可，无需 Unitree SDK 或 CUDA。

```bash
cmake -S . -B build \
  -DBUILD_SIMULATION=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime \
  -DBITBOT_MUJOCO_BUILD_TESTS=ON
cmake --build build --parallel 6
ctest --test-dir build --output-on-failure
```

Apple Silicon 可增加 `-DCMAKE_OSX_ARCHITECTURES=arm64`。本次适配的编译和
无窗口物理验证在 Linux 上执行；完整应用也完成了 10 秒窗口启动检查，现有两个
ONNX 网络和舞蹈轨迹正常初始化。策略运动效果、macOS 原生编译及窗口运行
仍需进一步验证。

本地源码包可通过 `-DBITBOT_MUJOCO_ARCHIVE=/path/to/mujoco-3.14.0.tar.gz`
指定，或放在 `dependencies/mujoco-3.14.0.tar.gz`。包应来自：
`https://github.com/google-deepmind/mujoco/archive/refs/tags/3.14.0.tar.gz`。
也可以使用 CMake 的 `FETCHCONTENT_SOURCE_DIR_MUJOCO` 指向已解压的同版本源码。
上述设置只提供 MuJoCo 源码；完全离线配置还需要提前缓存其第三方依赖。

完整应用仍按 `settings/CtrlConfig.json` 加载 ONNX/轨迹文件。保持既有相对路径时，
应从构建目录的 `bin` 运行程序：

```bash
cd build/bin
./main_app
```

其他构建目录应保持与 `build` 相同的目录层级；更深的目录需要调整模型和轨迹路径，
或改为绝对路径。本次独立验证在 `builds/mujoco-3.14` 完成，未覆盖原有 `build`。

## 接口变化

- 使用 `mujoco::libmujoco_simulate` 和新版 `Simulate(..., is_passive=false)`。
  窗口创建及渲染保留在主线程，物理循环在后台线程。
- 执行器命令通过 `actuator_ctrladr` 写入控制向量；Bitbot 的 `MujocoJoint`
  在 `mit` 模式接受 `input="pos vel ff"` 的无状态内置 PID；其余模式的模型映射
  只接受单输入执行器。反馈力矩通过 `actuator_outadr` 读取。
- 使用新版窗口库的 `InjectNoise`、加载提示及历史状态接口。重载模型后重新绑定
  设备，控制计时在仿真时间倒退时重新建立基准。
- 移除模型中的 `sensornoise` 标志及传感器 `noise` 属性。新版引擎不提供旧版的
  自动传感器噪声注入；需要噪声时应在观测处理层另行实现。
- `CtrlZ` 保持为独立子模块。本仓库提交不包含其 ONNX Runtime 平台链接改动；
  macOS 还需单独适配 `CtrlZ` 中写死的 Linux `.so` 链接路径。

## MIT 控制

当前 G1 模型使用 MuJoCo 内置 `<pid input="pos vel ff" ki="0">`，设备配置使用
`mode="mit"`。每个执行器在 `mjData.ctrl` 中占用三个输入，按顺序写入目标位置、
目标速度、前馈力矩。29 个执行器对应 87 个控制输入和 29 个力矩输出。

`MujocoJoint` 只写入这些目标值，不在 C++ 中计算 MIT 力矩；MuJoCo 在
`mj_forward` / `mj_step` 中根据当前物理状态计算：

```text
torque = Kp * (q_target - q) + Kd * (dq_target - dq) + tau_feedforward
```

`pos_kp` / `pos_kd` 提供初始增益。`MITMotorControlWorker` 可以在运行时更新两个
增益及三个目标值；适配层在物理线程写入 MuJoCo 3.14 的 `actuator_biasprm`
（`-Kp` 和 `-Kd`），由原生 PID 使用。MIT 模式不启用积分项或目标限速状态，
也不使用 `pos_ki`。XML 中的 PID 初始增益与现有设备配置一致。

原模型的力矩 `ctrlrange` 已转换为 PID 的 `forcerange`，限制最终输出力矩，
不会将角度或速度误当作力矩限幅。目标速度和前馈力矩均为零时，控制律等价于
原先 `pos_ki=0` 的 position 模式。旧 position / velocity / torque 模式的单输入
模型映射仍可使用；绑定后的原生 PID 允许切换为 torque（仅前馈）或 none。
禁用设备或切换到 none 时，同时清零目标输入及引擎中的反馈增益。

这是内置 PID 对理想 MIT 控制律的实现，没有电机电流环、通信延迟或热模型。
增益数值仍沿用现有仿真配置，不代表已与真机参数和运动表现一致。

兼容性测试无窗口运行：检查原生 PID 的三个输入、当前物理状态反馈、动态增益、零增益、力矩限幅及禁用行为，
检查多输入执行器前后的控制索引、拒绝错误设备映射，
并使用实际 G1 模型和 Bitbot 设备配置进行 500 步物理/传感器读写检查。
它不证明已训练策略在新物理引擎下的步态、稳定性或性能与旧版相同。
