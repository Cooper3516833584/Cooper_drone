# Drone Mission Control Framework

> 树莓派 4B (2GB) + ArduCopter (Matek H743 Slim V3) + MAVLink (UART) + Python 上位机控制框架

## 硬件前提

| 组件 | 说明 |
|------|------|
| 上位机 | 树莓派 4B (2GB), Raspberry Pi OS Lite 64-bit, 无桌面 |
| 飞控   | Matek H743 Slim V3, 刷 ArduCopter |
| 通信   | 树莓派 UART <-> 飞控 UART, MAVLink 协议 |

## 接线说明

```
树莓派 GPIO14 (TX) --> 飞控 RX (UART)
树莓派 GPIO15 (RX) <-- 飞控 TX (UART)
共地 GND --------------- GND
```

> **串口配置提醒**：树莓派需禁用串口控制台并启用硬件 UART：
> ```bash
> sudo raspi-config  # Interface Options -> Serial Port -> No login shell, Yes hardware
> ```
> 确认 `/dev/ttyAMA0` 可用（而非 `/dev/ttyS0` mini UART）。

## RC 通道含义

| 通道 | 功能 | PWM 含义 |
|------|------|----------|
| CH5  | 飞行模式/接管开关 | >= 1700 = 允许上位机接管 (Guided)，< 1700 = 人手 (Loiter) |
| CH7/8 | 紧急停桨 | >= 1700 = 紧急停桨（最高优先级，上位机立刻停止控制） |

## 安全设计

1. **KILL 最高优先级**：CH7/8 触发后，上位机立刻激活输出门禁 + 停止零速度 + 取消任务
2. **撤销接管**：CH5 切回 Loiter，上位机激活门禁 + 停止输出 + 飞手接管
3. **断链策略**：心跳超时后激活门禁 + 按配置执行 LAND 或 LOITER + 取消任务
4. **输出门禁（inhibit gate）**：安全事件触发后，control 层拒绝所有运动控制指令
5. **并发安全**：control.py 全局 RLock，所有控制原语串行执行
6. **任务可取消**：CancelToken 机制确保任何时刻可安全退出
7. **底层稳定**：上位机只做高层任务，姿态稳定永远交给飞控

## 安装

```bash
# 1. 创建虚拟环境
python3 -m venv drone_env
source drone_env/bin/activate

# 2. 安装依赖
pip install dronekit pymavlink opencv-python-headless flask PyYAML python-dotenv

# 3. 进入项目目录
cd drone_mission
```

## 启动命令

```bash
# 真机连接
python -m src.main --config config/vehicle.yaml --mission takeoff_and_hover

# Dry-run 模式（无需真机，使用 FakeVehicle 模拟）
python -m src.main --config config/vehicle.yaml --dry-run --mission takeoff_and_hover

# 带视频推流
python -m src.main --config config/vehicle.yaml --dry-run --enable-stream

# 使用启动脚本
chmod +x scripts/run.sh
./scripts/run.sh --dry-run --mission takeoff_and_hover
```

### 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--config PATH` | 配置文件路径 | `config/vehicle.yaml` |
| `--mission NAME` | 任务名称 | 无（待机模式） |
| `--dry-run` | 使用 FakeVehicle 模拟 | 否 |
| `--enable-stream` | 启用 Flask MJPEG 推流 | 否 |

### 可用任务

| 名称 | 说明 | 状态 |
|------|------|------|
| `takeoff_and_hover` | 起飞至指定高度并悬停 | 占位 |
| `waypoint_square` | 正方形航点飞行 | 占位 |
| `vision_track` | 视觉跟踪目标飞行 | 占位 |

## 日志位置

运行时日志自动生成到 `logs/drone_mission.log`，支持按大小滚动（默认 10MB x 5 个备份）。

## 主状态机流程

```
INIT -> CONNECT -> PRECHECK -> WAIT_RC -> RUN_MISSION -> SAFE_EXIT
  |        |          |          |           |
  +--失败--+----------+----------+-----------+--> SAFE_EXIT
```

**WAIT_RC 安全保障**：等待期间始终检查 KILL 和 LINK_LOST，断链立即退出（不会无限卡住）。

## 目录结构

```
drone_mission/
├── config/vehicle.yaml       # 所有常量配置
├── src/
│   ├── main.py               # 主状态机入口
│   ├── config_loader.py      # 配置加载
│   ├── logx.py               # 日志初始化
│   ├── types.py              # 共享数据结构
│   ├── fc_link.py            # 飞控连接管理
│   ├── safety.py             # 安全监控（RC/KILL/断链 + failsafe 动作 + 输出门禁）
│   ├── control.py            # 控制原语（全局锁 + 输出门禁）
│   ├── mission_base.py       # 任务基类与 CancelToken
│   ├── telemetry.py          # 遥测采样与发布（深拷贝）
│   ├── fake_vehicle.py       # FakeVehicle 模拟器（dry-run 用）
│   ├── vision/
│   │   ├── camera.py         # 摄像头采集
│   │   ├── tracker.py        # 目标检测/跟踪
│   │   └── stream_flask.py   # Flask MJPEG 推流（FPS 可配）
│   └── missions/
│       ├── takeoff_and_hover.py
│       ├── waypoint_square.py
│       └── vision_track.py
├── scripts/run.sh            # 启动脚本
├── systemd/                  # systemd 服务文件
├── tests/                    # 单元测试
└── logs/                     # 运行时日志
```

## 开机自启（可选）

```bash
sudo cp systemd/drone_mission.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable drone_mission
sudo systemctl start drone_mission
```
