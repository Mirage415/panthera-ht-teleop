set positional-arguments

default:
    @just --list

# Leader -> follower 遥操作；Ctrl+C 停止
Teleop *args:
    @"{{justfile_directory()}}/.venv/bin/python" -u "{{justfile_directory()}}/teleop.py" "$@"

# 仅查询通信和版本，不发送运动指令
Check *args:
    @"{{justfile_directory()}}/.venv/bin/python" -u "{{justfile_directory()}}/teleop.py" --check "$@"

# 读取 SDK 关节状态，退出时制动
State *args:
    @"{{justfile_directory()}}/.venv/bin/python" -u "{{justfile_directory()}}/teleop.py" --state "$@"

alias teleop := Teleop
alias check := Check
alias state := State

# 主臂诊断基线：手扶静态采样 8 秒，不提供重力支撑
CalibrateSample:
    @"{{justfile_directory()}}/.venv/bin/python" -u "{{justfile_directory()}}/teleop.py" --sample --role leader

# 会施加力矩：仅在现场托稳主臂后使用，8 秒结束
GravityObserve:
    @"{{justfile_directory()}}/.venv/bin/python" -u "{{justfile_directory()}}/teleop.py" --gravity-observe --role leader

Test:
    @"{{justfile_directory()}}/.venv/bin/python" -m unittest -v

# 启动预览服务，可在 Foxglove 页面开始/停止采集，不启动机械臂
Preview *args:
    @"{{justfile_directory()}}/.venv/bin/python" -u "{{justfile_directory()}}/capture.py" --preview "$@"

# 采集 RGB + follower 关节角；另一个终端运行 Teleop
Capture *args:
    @"{{justfile_directory()}}/.venv/bin/python" -u "{{justfile_directory()}}/capture.py" "$@"

# 常驻 Foxglove 预览与页面采集服务
alias Foxglove := Preview
