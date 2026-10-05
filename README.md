# Panthera-HT Teleoperation

基于高擎官方 SDK 的 Panthera-HT 主从遥操作工具。支持两块独立 USB/CAN 通信板、关节空间跟随、夹爪同步，以及单臂诊断。使用 `just Teleop` 启动。

**状态：实验性、正在实机调试。不是已经完成标定的控制系统。** 已验证两臂各 7 个电机的通信和状态读取，并进行了人工遥操作测试。主臂存在用户报告的关节上顶，重力模型、额外手柄负载与零位对应尚未完整验证；没有自动松手锁定。当前参数仅是这套硬件的候选试调值。

## 功能与边界

| 项目 | 当前实现 |
|---|---|
| 关节跟随 | MIT 模式位置/速度/前馈力矩/Kp/Kd，关节空间映射 |
| 主臂手感 | 模型重力补偿＋小阻尼，摩擦补偿关闭 |
| 从臂 | PD 跟随＋模型重力补偿，滤波速度前馈 |
| 目标限速 | 六关节 1.5 rad/s；夹爪 4.0 rad/s |
| 速度前馈 | 目标速度的 50%，40 ms 滤波；关节上限 0.75、夹爪 2.0 rad/s |
| 运行保护 | 状态时效/故障码/角度/目标跟踪误差/USB 变化检查，250 ms 电机 watchdog |
| 诊断 | 状态采样、8 秒主臂补偿观察、JSONL 日志 |
| 尚未实现 | 无按钮松手识别与位置保持、已验证的完整重力/摩擦标定、碰撞检测、力传感器反馈 |

目标限速不等于真实电机速度硬限幅，前馈力矩限幅也不等于 MIT 控制器总输出力矩硬限幅。本程序是普通 Linux/Python 控制循环，不是硬实时或经认证的安全控制器。软件测试不能替代实机验证。

## 来源

官方 SDK：[HighTorque-Robotics/Panthera-HT_SDK](https://github.com/HighTorque-Robotics/Panthera-HT_SDK)，以 Git submodule 固定在 `3712b3a3c2a4e6cbbc836a5d0b228a79d1a185c3`。遥操作参考 `panthera_python/scripts/5_teleop_control.py`，使用官方模型和电机 SDK。包装层单独维护，不修改上游文件。MIT 许可，版权见 [LICENSE](LICENSE)。

## 安装

已验证环境：Ubuntu 22.04 x86_64、Python 3.10。其他系统与架构未验证。

先安装 Git、Python 3.10、[uv](https://docs.astral.sh/uv/getting-started/installation/) 和 [just](https://github.com/casey/just#installation)，然后：

```bash
git clone --recurse-submodules https://github.com/Mirage415/panthera-ht-teleop.git
cd panthera-ht-teleop
bash setup.sh
```

安装创建本地 `.venv`，不修改系统 Python。Python 依赖固定在 `requirements.lock.txt`，电机 wheel 由固定版本的 SDK 提供。官方 wheel 文件名与内部版本号不同，安装脚本只针对该 wheel 启用 uv 的文件名兼容选项。

## 确认设备和权限

1. 同时连接两台臂，运行 `ls -l /dev/serial/by-path/`。
2. 在无控制程序运行且机械臂已稳妥承托时，拔掉 leader 的 USB，对比消失的路径；插回原接口。
3. 将确认的两个 CAN 通道路径填入 `teleop-config.json`。初始化时从 `teleop-config.example.json` 复制；示例中的 USB 路径是占位符，不能直接运行。
4. 如需查询各板的电机通道，可使用 `probe_ports.query(path)` 进行版本查询；不要凭 tty 编号猜测主从。每个选定通道必须返回电机 ID 1–7。
5. 两板均连接后配置串口权限：

```bash
sudo bash setup-device-access.sh
```

权限规则根据本机配置及 sysfs 验证生成，仅给当前 sudo 登录用户授权指定两块 Livelybot 板。换电脑 USB 插口后需重新核对路径及规则。

板卡可能使用相同序列号，因此不能只靠 `/dev/serial/by-id` 区分。运行时根据 `/dev/serial/by-path` 解析 tty 设备。底层 SDK 按设备名前缀枚举；如果出现 `ttyACM1` 与 `ttyACM10` 这类前缀歧义，启动器会拒绝运行。尚未提供自动解决这类枚举歧义的补丁。

## 使用

```bash
just Check                    # 查询电机版本、端口和依赖，不构造 SDK Robot
just State                    # 通过 SDK 读取状态，退出时制动
just State --role leader      # 单独主臂
just Teleop                   # 主从遥操作
just Teleop --duration 10      # 10 秒后退出
just Test                     # 软件测试，不打开硬件
```

`just teleop`、`just check`、`just state` 为小写别名。命令从仓库目录或其子目录运行，不需要激活虚拟环境。修改配置后需退出再重启；不热更新运行中的控制参数。

启动前固定底座，清空运动范围并扶稳主臂。退出用 `Ctrl+C`。**退出制动/超时阻尼不保证抗重力支撑，退出前扶稳两臂。** 启动目标从当前从臂位置开始，逐步追踪主臂；不会自动重设零位。由于尚未完成重力标定，不应将当前配置视为已消除漂移。

SDK 构造会进行通信初始化、可能重置 CAN，并在析构/退出时发送制动。因此 `State` 和采样不是完全无副作用的底层读取；只有 `Check` 不构造 Robot。

## 标定进度

目前完成的是诊断和人工试调，不是自动参数辨识：

```bash
just CalibrateSample  # 手扶静态采样 8 秒，不主动提供重力支撑
just GravityObserve   # 会施加力矩：主臂渐增补偿，最多 8 秒，需持续承托
```

观察阶段运动超过角度变化 0.15 rad 或速度 0.5 rad/s 会退出。日志包含触发保护的状态。手扶会改变关节外力，电机反馈力矩接近命令值不能证明重力模型准确。不要用这些数据直接拟合完整质量、重心或摩擦模型。

当前候选 `leader_gravity_scale=[1,1,0.95,0.90,1,1]`，即第 3 关节 95%、第 4 关节 90%。第 3 关节仍收到自行上抬反馈，尚未解决；主臂额外安装手柄，官方模型未针对本机负载完成修正。后续应核对真实姿态与模型零位、手柄质量与重心，再在多个姿态验证。**没有自动松手检测或保持模式。**

## 日志与退出状态

- `.runtime/teleop-latest.jsonl`：约 20 Hz，记录两臂状态、目标、补偿与主要参数；旧日志自动归档。
- `calibration/`：静态基线及重力观察数据。
- `.runtime/`：生成的 SDK 配置、锁文件、心跳与退出清理标记。

此前 SDK 退出清理曾被误报为控制心跳超时。现在分别判断运行心跳与清理阶段；SDK 清理超过 3 秒仍会报告清理超时并终止子进程，此限制不代表底层析构问题已修复。退出修正已软件测试，未重新进行硬件验证。

以上本机数据、真实端口配置、虚拟环境和下载产物不提交到 GitHub。公开示例保留控制参数，但不包含本机设备路径。

## 验证

单元测试覆盖状态超时/故障/非有限数据、目标限速、速度前馈停止和反向、限位、SDK 拒绝命令、USB 变化、补偿系数、退出状态以及权限规则。GitHub Actions 模板在 `ci/github-actions-test.yml`，尚未启用。当前发布凭据缺少 `workflow` 权限；有相应权限后将其移至 `.github/workflows/test.yml` 即可在无硬件环境自动执行测试。真实运动、断线和故障处置仍需独立的现场验收。
