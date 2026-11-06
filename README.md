# Panthera-HT Teleoperation

## 来源

基于高擎官方 [Panthera-HT_SDK](https://github.com/HighTorque-Robotics/Panthera-HT_SDK)，参考 `panthera_python/scripts/5_teleop_control.py`，封装 Panthera-HT 主从遥操作。

## 启动

在项目目录、环境和设备配置完成后运行：

```bash
just Teleop
```

`Ctrl+C` 停止，退出前请扶稳两臂。

## RealSense / Foxglove 采集

```bash
uv pip install --python .venv/bin/python -r requirements-capture.txt
just Preview                       # 常驻预览，在 Foxglove 页面点击开始/停止采集
just Capture                       # 开始一个新 episode，Ctrl+C 结束
# 页面点击 /capture/start 会自动启动 Teleop；终端手动遥操作仍可用：
just Teleop
```

打开 Foxglove，选择 **Open connection → Foxglove WebSocket**，连接
`ws://127.0.0.1:8765`。添加 Image 面板，选择 `/camera/color/image`；添加 Plot
面板，选择 `/follower/joint_states.position[0]` 至 `[5]`（六个关节），
`position[6]` 为夹爪电机角度，单位均为 rad。Raw Messages 面板选择
`/capture/status`，查看 `follower_live`、样本数、序号丢失数。

采集进程不打开机械臂串口。它接收现有 Teleop 控制循环每轮发送到本机 UDP
9877 的实测反馈，目标频率 100 Hz，实际频率取决于控制循环。预览时仅有画面，状态显示“等待/断流”，不会伪造或重复填充关节数据。
页面“开始”会自动启动 Teleop，等本次进程的反馈到达后录制；页面“停止”会结束两者。
手动从终端启动的 Teleop 不归页面管理，仍需在它自己的终端停止。不要同时启动第二个采集实例（UDP 端口排他绑定）。

每次采集保存到 `recordings/<UTC时间>/`：

- `data.mcap`：JPEG RGB、关节实测位置/速度/力矩及反馈时间戳、采集状态；拖进 Foxglove 回放。
- `joints.csv`：六个关节和夹爪的角度（rad）、速度（rad/s）、力矩（Nm）和反馈时间。
- `metadata.json`：相机型号、序列号、内参、时间策略、样本数及完成标记。

`just Capture --depth` 增加原始 16 位深度流 `/camera/depth/image`，
深度米值 = 原始值 × metadata 中的 `depth_scale_m`。深度与 RGB 保留各自坐标系，
未做对齐；当前不输出点云。`just Capture --joints-only` 仍实时显示图像，
文件只记录关节及状态。`--duration 30` 可定时结束，`--serial SERIAL` 可选择相机。
默认 RGB 为 640×480、30 fps，JPEG 质量 85。

图像使用主机收到帧的 Unix 时间，关节使用读取 SDK 状态后的主机 Unix 时间，
MCAP 和 CSV 保留纳秒整数。`/camera/frame_info` 额外保留设备时间及时间域，
关节消息保留七个电机各自的 SDK 反馈时间。这里是软件时间关联，**不是硬件同步**；
不假设 30 Hz 图像与约 100 Hz 关节一一对应，也不自动插值。
UDP 非阻塞发送避免磁盘/图像处理阻塞控制循环；可能丢包，序号用于检测接收期间的缺口。
MCAP 保存完整关节数组，CSV 的整数时间戳请用整数读取，避免浮点精度损失。

点击停止或正常退出会完成 MCAP 索引；无关节样本或无 RGB 帧的录制标为不完整。
服务退出时，若最后一段录制不完整则返回退出码 2。
`complete` 只表示正常结束且两类数据都有，不表示没有中途断流；请同时检查状态和缺口。
服务默认监听 `0.0.0.0:8765`。跨机器查看时，Foxglove 直接连接
`ws://<采集机局域网IP>:8765`；两台电脑在同一 Tailscale 网络时也可使用采集机的 Tailscale IP。
浏览器询问本地网络访问权限时允许。`0.0.0.0` 是监听地址，不是客户端连接地址。
如需仅允许本机连接，使用 `--host 127.0.0.1`。


## 在 Foxglove 页面控制采集


推荐使用 **Panthera Collection** 自定义面板，保留现有 Foxglove 连接、相机和关节 Plot。
扩展源码位于 `foxglove-control/`；构建后的 `.foxe` 拖进 Foxglove，再 Add panel →
Panthera Collection。也可由组织管理员上传至 Organization Settings → Extensions。
不需要导入布局 JSON。

面板内新建任务集、新建子任务（精确 prompt），然后按
**Start → Stop（可选）→ Save / Discard → Reset → 下一段** 操作。
任务目录和当前选择持久化到 `recordings/.collection-workflow.json`。
录制时不可切换任务；每段 `metadata.json` 固定保存任务集 ID、名称、子任务 ID 和 prompt。
LeRobot 的 task 文本取该段 prompt，来源清单保留任务归属；历史数据不重新标注。
现有 WebSocket 地址、录制目录和 Hugging Face 仓库保持不变。

**Reset 是实际自动回位**：新采集在第一个录入的关节样本中保存主从臂及夹爪姿态。
Save/Discard 后分别点击 Reset Leader / Reset Follower，各自回到本轮实测起始姿态。
单臂回位只连接所选机械臂，不要求 Teleop 运行或另一只臂在线；回位期间不录制，可用 Stop 中止。
两臂完成状态分别保存，两者都完成后允许下一段。旧 `/capture/reset` 双臂回位接口仍可用。
限位外不超过 0.01 rad 的起始读数投影到合法边界，运动目标始终位于原有关节限位内；超出此容差拒绝回位。
回位状态与错误单独通过 `reset_state` / `reset_error` / `reset_role` 输出。
缺少起始姿态的历史 episode 不能自动回位。回位走同一个 supervisor、进程锁、端口预检、
反馈/故障检查、心跳和 watchdog；关节参考速度最多 0.35 rad/s、夹爪 0.5 rad/s，
跟踪误差超过 0.3 rad（或原配置更小阈值）停止，45 秒未收敛也停止。
全部轴误差 ≤0.04 rad 且速度 ≤0.08 rad/s 持续 0.5 秒才确认完成。
成功后退出运动进程，沿用现有阻尼制动，并不持续位置保持；请承托重力负载。
Reset 不会自动重摆场景物体。未完成回位时不能 Start 下一段。

面板显示 Teleop 状态、follower 反馈新鲜度、相机帧率、帧龄，以及估计的
**相机 → 采集机延迟**。延迟仅使用 RealSense `global_time` 映射到主机时钟的帧时间，
不包含网络、JPEG 编码或浏览器渲染；时间域不可比较或读数异常时显示不可用。
新控制接口 `/capture/command` 使用动作及状态 revision，拒绝旧页面/旧状态的动作；
Stop 即使在状态过期时仍可发送。原有单独 Service Call 接口继续保留。


启动一次 `just Preview`（也可用 `just Foxglove`），保持服务运行。
连接后，在当前布局添加四个 **Service Call** 面板：

| 设置 | Start | Stop | Save | Discard / Reject |
| --- | --- | --- | --- | --- |
| Service name | `/capture/start` | `/capture/stop` | `/capture/save` | `/capture/discard` |
| Request | `{}` | `{}` | `{}` | `{}` |
| Button → Title | Start | Stop | Save | Discard |
| Timeout | 10 秒 | 10 秒 | 10 秒 | 10 秒 |

Start 启动遥操作及录制；Stop 停止并暂存待审核；Save 保留、转换并上传；Discard 排除当前 episode。
Save 和 Discard 可在录制中直接点击，会先停止遥操作和关闭文件。
Discard 为可恢复的本地标记，不删除原始文件，但不会进入训练集或上传。
已经 Save 的 episode 不能再通过 Discard 撤销，以免误改正在上传或已发布的数据。

配置好后可关闭 **Editing mode**，面板只显示按钮；首次使用建议保持开启，
直接查看响应中的 `ok`、`message` 和 `episode_path`。将按钮放在 Image 面板旁，保存布局。
服务由 Foxglove SDK 原生提供，不需要额外网页、插件或 HTTP 端口。

再添加 **Raw Messages** 面板，topic 选择 `/capture/status`，实时显示：

- `recording` / `recording_state`：是否正在录制（idle / recording / error）。
- `elapsed_s`：当前 episode 时长。
- `episode_statistics`：当前 episode 的图像、关节样本数和序号缺口。
- `episode_path`：正在保存的目录（采集机本地路径）。
- `last_episode_path` / `last_episode_complete`：上一段文件位置及完成标记。
- `camera_live` / `follower_live`：数据是否在最近 0.5 秒内更新。
- `recording_error`：写入或关闭文件失败的信息。
- `teleop_state`：idle / starting / running / stopping / error。
- `teleop_error` / `teleop_log`：启动失败或异常原因以及本机日志路径。
- `review_status` / `pending_review`：recording / pending / saved / discarded，以及是否等待选择。
- `upload_state` / `upload_error` / `upload_url`：上传状态、错误和 Hugging Face 链接。
- `upload_pending` / `upload_completed` / `upload_failed`：上传队列计数。

页面开始按钮要求相机在线，随后自动启动现有 `teleop.py`（与 `just Teleop` 相同配置和预检）。
按钮立即返回 `starting`，首次反馈就绪后状态自动变为 `running` 并开始保存，无需另开终端。
完整流程是：现场准备好机械臂 → 页面点击开始 → 等待 `running` → 操作 → 页面点击停止。
停止会请求 Teleop supervisor 退出，走原有制动和 watchdog 流程，再关闭数据文件；请在停止时扶稳两臂。
启动最多等待 30 秒；预检失败、反馈断流或保存失败会停止本次进程并显示错误。
只有本次子进程的反馈能触发自动录制，已有手动 Teleop 通过原有进程锁拒绝重复启动。
页面不会自动接管或停止其他终端启动的 Teleop。
每次开始生成新目录；重复点击开始不会拆分当前 episode，重复停止不会覆盖文件。
点击停止后画面继续显示。先 Save 或 Discard 当前 episode，再 Start 下一段。
未审核 episode 会跨服务重启保留；不完整的 episode 不能 Save 为训练数据，可 Discard。

关闭网页或网络断开不会自动停止录制。重连后可查看状态并点击停止。
`Ctrl+C` 退出采集服务时会停止其启动的 Teleop 并关闭当前 episode。
服务异常会将当前 episode 标为不完整；Teleop supervisor 检测到所属采集服务退出时也会停止 worker。
强制断电或 SIGKILL 无法保证 MCAP 索引完成；创建时 metadata 的完成标记为 false。

`just Capture` 仍保留终端直接开始录制的行为（可先于 Teleop 启动），支持页面停止录制；若要之后用页面自动启动 Teleop，请先退出手动 Teleop。
`--preview` 现在表示启动时只预览，收到页面开始命令后才保存。


## Hugging Face / LeRobot

目标仓库由本机 `hf-config.json` 指定。当前配置为 `Mirage415/nyush_panthera_test`，
仓库保持用户原有可见性。新建仓库默认 private；现有仓库不修改可见性。
只有 `review_status: saved` 且 `complete: true` 的 episode 会后台转换并上传。
上传只包含 LeRobot 的 `data/`、`meta/`、`videos/`、说明及来源清单，不上传本机日志或凭据。
原始 MCAP / CSV 保留在采集机。

转码使用独立 `.venv-lerobot` 环境，避免升级机械臂 SDK 所依赖的 NumPy。
重建环境：

```bash
uv venv --python 3.12 .venv-lerobot
uv pip install --python .venv-lerobot/bin/python -r requirements-lerobot.lock.txt \
  --extra-index-url https://download.pytorch.org/whl/cpu --index-strategy unsafe-best-match
```

LeRobot 官方 writer 生成 30 FPS 数据，官方 reader 验证可读后才上传：

- `observation.images.realsense`：640×480 RGB 视频。
- `observation.state`：六个 follower 关节及夹爪电机实测角度，rad。
- `action`：经过限位、速度限制后实际下发的 follower 目标角度，rad。
- `source.*_timestamp_ns`：原始主机时间戳，int64 纳秒。

RGB 按最近帧匹配（最大 35 ms）；状态及 action 使用最近一次已发生的样本
（最大 50 ms / 80 ms），超限时不导出该 episode。不是硬件同步。
历史无 action 字段的录制，只有找到时间匹配的原始遥操作 target 日志才可恢复；
旧日志约 20 Hz，转换时保持最近指令，来源和最大匹配时间差写入 `source_episodes.json`。
新采集直接在每轮已成功下发控制指令后记录目标。任务文本暂用通用 `Panthera teleoperation`，
不是人工任务标注；可在 `hf-config.json` 中修改 `task` 供之后保存的 episode 使用。

上传是独立后台进程，不阻塞画面或控制循环。失败保留本地文件，最多自动尝试三次，
间隔至少 60 秒。可增加 Service Call 面板，服务 `/capture/upload_retry`，请求 `{}`，手动重试。
队列保存在每段的 `upload.json`，转换详情在 `conversion.log`，服务重启后会继续处理。
重复上传不会重复追加 episode；已有远端来源清单与本机不一致时拒绝覆盖。
