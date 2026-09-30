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
  只接受单输入执行器。其他多输入执行器可以存在于模型中，但不能映射为
  `MujocoJoint`。
- 使用新版窗口库的 `InjectNoise`、加载提示及历史状态接口。重载模型后重新绑定
  设备，控制计时在仿真时间倒退时重新建立基准。
- 移除模型中的 `sensornoise` 标志及传感器 `noise` 属性。新版引擎不提供旧版的
  自动传感器噪声注入；需要噪声时应在观测处理层另行实现。
- `CtrlZ` 保持为独立子模块。本仓库提交不包含其 ONNX Runtime 平台链接改动；
  macOS 还需单独适配 `CtrlZ` 中写死的 Linux `.so` 链接路径。

兼容性测试无窗口运行：检查多输入执行器前后的控制索引、拒绝错误设备映射，
并使用实际 G1 模型和 Bitbot 设备配置进行 500 步物理/传感器读写检查。
它不证明已训练策略在新物理引擎下的步态、稳定性或性能与旧版相同。
