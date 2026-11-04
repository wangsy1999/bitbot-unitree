# 原始参考轨迹

- `supermimic/`：当前 checkpoint 训练使用的 40 条原始 CSV，30 FPS。
- `unitree_rl_lab/`：`dance_102` 和 `gangnam_style` 的完整原始 CSV，60 FPS。
- `custom/`：本仓库原有的 `jiaqi_KuaileChongbai2.csv`，50 FPS，完整时长 57.92 秒。
- `manifest.json`：动作名称、相对路径、原始帧率、裁剪范围、SHA256 和来源。
- `training_run_config.json`、`unitree_config.yaml`：原始训练/播放配置快照。

CSV 均保留源文件内容。每行是 root position XYZ、root quaternion XYZW、29 个关节位置，
关节按设备 ID 顺序排列。`gangnam_style` 的运行参考取原始 5.2–25.5 秒区间，
程序直接读取这里的完整 CSV，并按 manifest 的 Range 配置取有效片段。
启动时重采样及计算参考速度，推理时生成未来目标，不依赖二进制轨迹。

manifest 中的 `Path` 相对本目录；上级 `worker.json` 的 `Source.Motions`、配置路径
及 manifest 路径相对 `worker.json` 所在目录。`OriginalPath` 等原始路径仅作来源记录。
修改 CSV 或 manifest 后重启程序即可生效，不需要重新导出模型。
更换模型权重时，导出工具支持 `--manifest` 复用本清单。
