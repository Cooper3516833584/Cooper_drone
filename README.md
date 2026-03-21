# Cooper Drone (pymavlink-only)

本项目是树莓派侧无人机控制骨架，面向 ArduPilot 飞控。

## 关键原则

- 只使用 `pymavlink`，不依赖 DroneKit
- 任务层不直接操作 MAVLink 原始消息
- 所有飞行动作统一经过 `src/control.py`
- 安全线程独立运行，不依赖任务循环是否正常

## 当前实现状态

### 已完成

- 会话层：`MavSession` / `FakeMavSession`
- 状态层：线程安全 `VehicleStateCache`
- 遥测层：`TelemetryHub` 周期发布快照
- 控制层：模式/解锁/起飞/降落/goto/机体系速度控制
- 安全层：KILL、接管撤销、断链、RC stale
- 最小可执行任务：`takeoff_and_hover`
- `--dry-run` 主流程与测试

### 未完成（本轮故意保留占位）

- `waypoint_square` 高级航点逻辑
- `vision_track` 视觉闭环控制
- 真实视觉识别算法与策略融合

## 目录

```text
src/
  config_loader.py
  logx.py
  types.py
  mav_state.py
  mav_session.py
  fc_link.py
  telemetry.py
  control.py
  safety.py
  mission_base.py
  main.py
  missions/
    takeoff_and_hover.py
    waypoint_square.py
    vision_track.py
  vision/
    camera.py
    tracker.py
    stream_flask.py
tests/
  test_config_loader.py
  test_control_fake_session.py
  test_mission_build.py
  test_main_dry_run.py
  test_safety_manager.py
  test_preflight.py
```

## 配置说明

主配置文件是 `config/vehicle.yaml`，包含：

- `mavlink`: 链路与消息频率、ACK 超时、重试
- `rc`: 接管通道、kill 通道、RC stale 超时
- `limits`: 起飞高度、速度/角速度限幅、到点接受半径
- `failsafe`: 断链与撤销接管动作（LAND/LOITER/BRAKE）
- `telemetry`: 遥测采样频率
- `safety`: 安全轮询频率

环境变量可覆盖部分配置（见 `.env.example`）：

- `COOPER_MAVLINK_PORT`
- `COOPER_MAVLINK_BAUD`
- `COOPER_LOG_LEVEL`

## 任务开发约束

任务开发者应只使用：

- `ctx.session`
- `ctx.telemetry`
- `ctx.cancel`
- `control.py` 控制原语

任务层禁止直接调用：

- `recv_match`
- MAVLink 原始消息构造
- ACK 处理逻辑

## 当前可执行任务

- `takeoff_and_hover`：可运行
- `waypoint_square`：占位，抛 `NotImplementedError`
- `vision_track`：占位，抛 `NotImplementedError`

## dry-run 运行

```bash
python -m src.main --config config/vehicle.yaml --dry-run
python -m src.main --config config/vehicle.yaml --dry-run --mission takeoff_and_hover
```

## 真机串口运行

```bash
python -m src.main --config config/vehicle.yaml --mission none
python -m src.main --config config/vehicle.yaml --mission takeoff_and_hover
```

## 测试

```bash
pytest -q
```

## 安全逻辑摘要

- KILL 触发：取消任务 + 抑制运动输出 + 停控 + 强制上锁
- 接管撤销：取消任务 + 抑制运动输出 + 停控 + 切安全模式
- 心跳丢失：取消任务 + 抑制运动输出 + 执行一次断链动作
- RC stale：进入保守状态，不再继续允许 guided 接管

