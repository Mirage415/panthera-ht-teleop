# Panthera-HT 双臂 Viser 控制台

基于高擎官方 Panthera-HT SDK。两只臂均使用 Follower 硬件配置；网页控制和 A→B 主从遥操是运行模式。

统一入口：**http://10.209.66.60:8082/**。启动仅连接反馈，不自动运动。

## 页面

- **机械臂**：模式、开始、双臂复位；关节角度和高级参数折叠。
- **相机**：主画布大图，彩色/深度切换，USB、帧率与帧龄。
- **采集**：任务集/任务、开始采集、停止录制、保存/丢弃。任务管理、详情、历史回放默认折叠。
- **数据**：A/B 六关节及夹爪，位置、速度、力矩曲线，实测/目标/参考、末端坐标。
- **日志**：当前及历史故障、启动和停止原因。

全局“停止”停止机械臂，正在录制的文件随后结束。采集页“停止录制”只结束文件，机械臂控制继续。运行中可直接点“双臂复位”：六关节低速回零，完成后保持零位，不重设编码器、不自动合拢夹爪。

只保留回零，没有“本段起点”、单臂起点恢复或必须回位才能录下一段的流程。

## 采集流程

1. 在折叠的“任务管理”里创建任务集与精确任务描述，或选择已有任务。
2. 相机和两臂反馈就绪后点“开始控制并录制”；若控制已运行，按钮为“开始录制”，沿用当前模式。
3. 点“停止录制”，或直接“保存/丢弃”。保存保留本段；丢弃仅标记排除，不删除原始文件。
4. 处理完上一段即可录下一段，不强制回零。

采集开始时若处于待机/零位保持，会按机械臂页所选模式启动控制；请确认现场已准备好。未连接、旧页面请求、已断开的窗口不会启动。

每段保存在 `recordings/<UTC时间>/`：

- `data.mcap`：RGB JPEG、可选原始 16 位深度 PNG、两臂完整位置/速度/力矩与反馈时间、控制目标、相机设备时间信息。
- `joints.csv`：B 臂六关节与夹爪、时间戳及可用的真实跟随目标；A/B 完整数据在 MCAP 中。
- `metadata.json`：任务快照、相机参数、时间策略、样本/缺口统计、完成及审核状态。

电机循环以约 100 Hz 非阻塞投递双臂数据；相机服务以配置帧率保留一致的 RGB/深度帧组。队列或相机缓冲溢出会记录序号缺口，不补造样本。图像与反馈使用主机纳秒时间，额外保留设备时间；没有硬件同步。相机采集延迟只在设备 global_time 可与主机时间比较时提供，不含编码/网络/显示。

RGB 与深度保留各自光学坐标系，未做像素对齐。深度米值 = PNG 原始值 × metadata 的 depth_scale。相机提供彩色/深度内参与内部外参；机器人外参尚未标定，Viser 底座位置仍为展示布局。

主从模式记录 B 臂真实七轴目标，保留原 LeRobot 导出路径。网页控制记录双臂六关节参考/目标与夹爪反馈；因为夹爪没有位置目标，不伪造七轴动作，这类录制用于原始分析/回放，现有七轴 LeRobot 导出会拒绝缺失 action 的数据。

历史与回放支持拖动时间进度，查看图像及双臂关节。支持旧 protobuf MCAP 与新的 JSON MCAP；回放不会控制实机。异常退出的未完成录制恢复为待审核且不完整，不能保存为训练数据。

## 安装与启动

```bash
./setup.sh
uv pip install --python .venv/bin/python -r requirements-viser.txt
just Preview                    # 模型预览，测试录制目录独立，不连接电机
just Viser                      # 实机控制台；不要与 just Teleop 同时运行
just Check                      # 原串口/版本诊断
just Test
```

相机服务由 `camera_service.py` 单独持有 RealSense USB。Viser 通过 HTTP 获取画面及有时间戳的帧组，不重复打开相机。NUC 上由 `realsense-web.service` 管理，8080 为相机服务，8082 为控制台。

`panthera-viser.service` 不自动使能。通信丢失、控制进程异常或控制浏览器断开会停止；重新连接不会自动恢复运动。停止为阻尼制动，不保证重力支撑。

## 导出与上传

沿用 `hf-config.json`（已忽略）和 Hugging Face 登录。只有审核为 saved 且完整的数据进入原后台导出/上传队列；上传状态、失败信息和重试入口在采集页详情中。没有配置时只保存本地，不上传。

LeRobot 使用独立 `.venv-lerobot` 和 `requirements-lerobot.lock.txt`，不修改机械臂环境的 NumPy ABI。`export_lerobot.py` 保留源时间戳和任务归属，检查对齐误差，失败不覆盖此前导出。

## 代码边界

- `viser_app.py`：场景、机械臂 UI、控制进程管理。
- `viser_worker.py` / `viser_control.py`：SDK 循环、状态机与非阻塞采样。
- `viser_collection.py` / `recording.py` / `mcap_io.py`：采集线程、文件生命周期、独立 MCAP 写入。
- `viser_collection_ui.py` / `viser_playback.py`：简洁采集页面、只读历史回放。
- `camera_service.py` / `viser_camera.py`：单一相机采集服务与画面展示。
- `collection_workflow.py` / `hub_upload.py` / `export_lerobot.py`：任务、后台上传、训练导出。

原外部可视化服务、前端扩展、布局文件和 SDK 依赖已移除；不再监听原来的 8765 端口。保留独立终端遥操及诊断功能，和 Viser 共用硬件排他锁。

## 相机参数保存

camera-settings.json 按设备序列号保存 RGB 设置（本机配置，Git 忽略）。相机服务每次启动或重新连接时加载，/api/status 的 color_options 返回加载后的实际读数。当前固定相机使用手动曝光 200、增益 64、自动白平衡；不复制到其他序列号的相机。

首次部署可复制 camera-settings.example.json 为 camera-settings.json，并将 YOUR_CAMERA_SERIAL 替换成实际相机序列号。
