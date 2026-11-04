# SuperMimic 独立程序

`supermimic_app` 的入口、types、user_func、user_command 和设备配置均位于
`src_supermimic/`。复用 CtrlZ/设备驱动，不依赖 camera 程序或 DWAQ 程序的用户代码。
默认在 `g1_camera` 分支的 MuJoCo 2.3.7 环境中构建。

## 构建和启动

```bash
cmake -S . -B build -DBUILD_SIMULATION=ON -DBUILD_SUPERMIMIC_APP=ON
cmake --build build --target supermimic_app -j4
./build/bin/supermimic_app
```

只构建此应用时，可加 `-DBUILD_CAMERA_APP=OFF -DBUILD_DWAQ_APP=OFF`。
控制沿用 Bitbot Copilot/遥控器事件：`9` 上电、`8` 运行、`p` 复位、`r` DWAQ 行走，
`D` 切换至 SuperMimic，`g` 从动作首帧开始播放。切换后未按 `g` 时，参考停在首帧。
`n` 选择下一个动作、`b` 选择上一个动作，首尾循环；可在复位、行走或 SuperMimic 状态选择。
选择后暂停在新动作首帧，清空历史和上一动作，终端显示动作名，按 `g` 开始。
`D` 会清空动作历史和上一动作；`g` 重新计时并清空历史。动作播放到末尾后参考保持最后一帧，
再次按 `D`、`g` 可重新播放。空格停止。遥控器映射见 `src_supermimic/settings/setting.json`。

## 模型和动作

默认导出自本机 Uni-SuperMimic 的 `2026-10-23_17-05-06_mjwarp/last.pt`，
迭代 12000，默认包含该 checkpoint 训练记录中的 **40 个动作**，
另外追加 Unitree RL Lab 的 `dance_102`、`gangnam_style` 和本仓库的
`jiaqi_KuaileChongbai2`，共 **43 个动作**。
前 40 个动作顺序与训练配置一致，所有动作共用当前 SuperMimic 策略模型。
训练动作源数据为 30 FPS，Unitree RL Lab 两条轨迹为 60 FPS；
`jiaqi_KuaileChongbai2` 为 50 FPS，共 2897 帧、57.92 秒。部署数据统一为 50 Hz。
导出来源、迭代和动作路径记录在 `CheckPoints/supermimic/worker.json` 的 `Source` 字段。
该字段仅作记录，运行时不需要原训练仓库或 checkpoint。
追加轨迹的原始帧率记录在 `Source.MotionSourceFPS`。
`dance_102` 使用完整片段，为 29.12 秒；`gangnam_style` 按 Unitree RL Lab 配置
取原始 5.2–25.5 秒区间，为 20.30 秒，从原始第 312 帧（从 0 计数）开始。
运行时裁剪范围以 `trajectories/manifest.json` 的 `Range` 为准，未来目标截断在裁剪终点。它们未使用 Unitree RL Lab 的模型，
当前模型对追加轨迹的跟踪效果尚未验证。

原始 CSV 已保存在 `CheckPoints/supermimic/trajectories/`：
`supermimic/` 中有 40 条训练轨迹，`unitree_rl_lab/` 中有 2 条追加轨迹，
`custom/` 中保存 `jiaqi_KuaileChongbai2.csv`。
`manifest.json` 记录动作名、原始帧率、裁剪区间和校验值，路径相对清单目录；
`worker.json` 的 `Source.Motions` 等本地来源路径相对动作包目录。
原始文件保留完整内容，`gangnam_style` 的裁剪在程序启动时执行。
详见该目录的 [README](../CheckPoints/supermimic/trajectories/README.md)。

导出包包含：

- `policy.onnx`：目标编码器、策略解码器、冻结的观测和目标归一化；不部署训练世界模型或 critic。
- `worker.json`：网络名、历史长度、目标帧数、关节顺序、默认姿态、动作缩放、PD 参数。
- `trajectories/manifest.json` 和 CSV：程序直接读取的动作列表、帧率、裁剪区间与原始轨迹。

程序启动时按源 MotionLib 约定重采样到 50 Hz，计算参考旋转、关节速度和局部基座速度。
推理时现场构造未来 10 帧目标，不生成或读取 `.bin`。启动完成后不再读取轨迹文件。
修改 CSV、帧率或裁剪区间后，重启程序即可生效，无需重新导出模型或编译。
清单中的 SHA256 仅作原始文件校验记录，运行时不强制校验。

修改 `src_supermimic/settings/CtrlConfig.json` 中
`Workers.DanceNet1.BundlePath` 可选择另一个包。相对路径按项目根目录解析，包内的模型和
参考路径按 `worker.json` 所在目录解析；绝对路径也可用。
`Workers.DanceNet1.InitialMotion` 设置启动时选中的动作名称。
动作顺序和名称由 manifest 中的 `Motions` 数组决定，均不写死在 C++ 中。DWAQ 行走模型独立配置。

用源项目 Python 环境重新导出（不启动任何仿真）：

```bash
/home/wsy/rl/Uni-SuperMimic/.venv/bin/python tools/export_supermimic.py \
  --source /home/wsy/rl/Uni-SuperMimic \
  --checkpoint /path/to/last.pt \
  --manifest CheckPoints/supermimic/trajectories/manifest.json \
  --output CheckPoints/supermimic
```

工具使用源项目的 Policy 和机器人参数，导出时做少量 ONNX/PyTorch 离线输出对照。
`--manifest` 保留现有动作名称、各自的帧率和裁剪区间；新增数据也可以使用
`--motion file1.csv file2.csv --source-fps 30`，文件会复制到动作包内并生成 manifest。
输入文件名去掉 `.csv` 后作为默认动作名称，名称不能重复。两种输入方式选择一种。
只有更换模型权重时才需要重新导出；修改轨迹不需要 Python 环境。

## 推理约定

CtrlZ worker 为 `CtrlZ/CtrlZ/Workers/NN/SuperMimicInferenceWorker.hpp`。
默认输入为 `obs_hist [1,10,99]`、`goal [1,730]`，输出 `actions [1,29]`。
历史从旧到新排列，初始化时用首帧填满。

单帧观测依次为：关节位置减默认位置 29、关节速度 29、机体系角速度 3、
机体系重力 3、上一原始动作 29、当前基座到参考基座的相对旋转 6。
6D 旋转按源代码取旋转矩阵前两列再按行展开。使用主 IMU 的原始 RPY（弧度）和角速度，
不使用 torso IMU，也不额外缩放或裁剪观测。归一化已包含在 ONNX 内。
部署采用零关节偏置上下文。

未来目标按源 MotionLib 的算法在 C++ 中生成，每帧 73 维，使用未来 1 至 K 帧，末尾截断到最后一帧。
进入动作、选择动作或开始播放时，把所选动作的初始朝向对齐到当前基座 yaw，保留参考 roll/pitch。
目标位置为 `default_position + action_scale * raw_action`，不额外裁剪。
所有输入输出及 PD 参数按设备 ID 与 CtrlZ 关节顺序重排。

## 仿真资产与验证范围

独立模型位于 `src_supermimic/models/`，来自源项目的 G1 MJCF，保留惯量、碰撞和 armature。
为适配当前 Bitbot/MuJoCo 2.3.7，将原生 position 执行器转换为限幅 torque motor，
由 Bitbot 的位置 PD 计算力矩；PD 参数由动作包配置。显式开启旧引擎的 autolimits，
移除旧引擎不支持的 mesh content_type 和关节 actuatorfrcrange，力矩限制保留在 motor 上。
复用内容一致的现有网格，补充 4 个源项目网格，新增主/副 IMU site。
仿真步长为 2ms，策略周期 20ms，保留当前框架时序。
来源及许可保留在 `src_supermimic/models/LICENSES/`。

已验证独立程序编译、模型静态加载和 ONNX/PyTorch 离线一致性；另外以两帧输入
对照了 C++ worker 与源 MDP/Policy 的观测历史、6D 旋转、未来目标、上一动作和映射后的关节目标。
多动作版本另外通过了选择、前后循环、首帧目标及历史/上一动作清空的离线检查。
CSV 读取版本对全部 43 个动作的首帧、中间帧和末帧进行了离线对照，
参考状态和未来目标与原 MotionLib 数据的最大差异约为 1.4e-6。
尚未验证完整动作仿真或真机稳定性；本次没有使用 Isaac Sim。

## 默认动作清单

编号从 1 开始，`n` 向下一个切换、`b` 向上一个切换，首尾循环。

| 编号 | 动作 |
| --- | --- |
| 1 | `dance1_subject1` |
| 2 | `dance1_subject2` |
| 3 | `dance1_subject3` |
| 4 | `dance2_subject1` |
| 5 | `dance2_subject2` |
| 6 | `dance2_subject3` |
| 7 | `dance2_subject4` |
| 8 | `dance2_subject5` |
| 9 | `fallAndGetUp1_subject1` |
| 10 | `fallAndGetUp1_subject4` |
| 11 | `fallAndGetUp1_subject5` |
| 12 | `fallAndGetUp2_subject2` |
| 13 | `fallAndGetUp2_subject3` |
| 14 | `fallAndGetUp3_subject1` |
| 15 | `fight1_subject2` |
| 16 | `fight1_subject3` |
| 17 | `fight1_subject5` |
| 18 | `fightAndSports1_subject1` |
| 19 | `fightAndSports1_subject4` |
| 20 | `jumps1_subject1` |
| 21 | `jumps1_subject2` |
| 22 | `jumps1_subject5` |
| 23 | `run1_subject2` |
| 24 | `run1_subject5` |
| 25 | `run2_subject1` |
| 26 | `run2_subject4` |
| 27 | `sprint1_subject2` |
| 28 | `sprint1_subject4` |
| 29 | `walk1_subject1` |
| 30 | `walk1_subject2` |
| 31 | `walk1_subject5` |
| 32 | `walk2_subject1` |
| 33 | `walk2_subject3` |
| 34 | `walk2_subject4` |
| 35 | `walk3_subject1` |
| 36 | `walk3_subject2` |
| 37 | `walk3_subject3` |
| 38 | `walk3_subject4` |
| 39 | `walk3_subject5` |
| 40 | `walk4_subject1` |
| 41 | `dance_102` |
| 42 | `gangnam_style` |
| 43 | `jiaqi_KuaileChongbai2` |
