"""
safety — 安全层，常驻监控（带 failsafe 动作与输出门禁）

职责：
    永远高优先级运行，随时打断上位机任务。
    负责 RC 通道监控、紧急停桨、接管撤销、断链策略。

安全策略（优先级从高到低）：
    1. **KILL 触发**（CH7/8 PWM >= kill_pwm_min）：
       - 立刻调用 control.inhibit_outputs() 禁止任务发控制指令
       - 立刻调用 control.stop_motion() 发送零速度
       - 取消所有已注册任务（CancelToken）
       - 进入 EMERGENCY 状态
       - 注意：真正停桨由飞控自身执行，上位机仅停止干预

    2. **撤销接管**（CH5 从 Guided 回到 Loiter）：
       - 立刻调用 control.inhibit_outputs() 禁止任务发控制指令
       - 立刻调用 control.stop_motion() 发送零速度
       - 取消所有已注册任务
       - 不再向飞控发送任何指令（让飞手接管）

    3. **心跳超时 / 断链**：
       - 立刻调用 control.inhibit_outputs() 禁止任务发控制指令
       - 按 failsafe.link_lost_action 执行：
         - LAND：set_mode_nowait("LAND")
         - LOITER：set_mode_nowait("LOITER")
       - 取消所有已注册任务

控制权设计：
    - CH5：Loiter = 人手控制，Guided = 允许上位机接管
    - CH7/8：紧急停桨（最高优先级）
    - 上位机只做"高层任务"，底层姿态稳定永远交给飞控
"""

from __future__ import annotations

import enum
import logging
import threading
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from src.config_loader import AppConfig
    from src.fc_link import FlightControllerLink
    from src.mission_base import CancelToken

logger = logging.getLogger(__name__)


class SafetyState(enum.Enum):
    """安全状态枚举。"""
    NORMAL = "NORMAL"                  # 正常（上位机无控制权）
    GUIDED_ALLOWED = "GUIDED_ALLOWED"  # RC 允许上位机接管
    KILL = "KILL"                      # 紧急停桨已触发
    LINK_LOST = "LINK_LOST"            # 飞控心跳丢失


class SafetyManager:
    """安全管理器——常驻后台线程，持续监控 RC 通道与心跳。

    安全事件触发时：
        1. 激活 control 层输出门禁（inhibit gate）
        2. 发送 stop_motion 停止运动输出
        3. 按策略执行 failsafe 动作（LAND/LOITER）
        4. 取消所有已注册的 CancelToken

    Usage::

        safety = SafetyManager(cfg, link)
        safety.start()
        ...
        if safety.is_guided_allowed():
            # 可以执行任务
            pass
        ...
        safety.stop()

    Args:
        cfg:  全局配置实例。
        link: 飞控连接管理器（用于读取 RC 通道和心跳）。
    """

    def __init__(self, cfg: AppConfig, link: FlightControllerLink) -> None:
        self._cfg = cfg
        self._link = link

        self._state = SafetyState.NORMAL
        self._lock = threading.Lock()
        self._cancel_tokens: list[CancelToken] = []
        self._running = False
        self._thread: threading.Thread | None = None

        # 配置缓存
        self._mode_ch: int = cfg.rc.mode_channel
        self._kill_ch: int = cfg.rc.kill_channel
        self._guided_pwm_min: int = cfg.rc.guided_pwm_min
        self._kill_pwm_min: int = cfg.rc.kill_pwm_min
        self._heartbeat_timeout_s: float = cfg.failsafe.heartbeat_timeout_s
        self._link_lost_action: str = cfg.failsafe.link_lost_action.upper()
        self._poll_interval_s: float = 0.1  # 10 Hz 监控频率

    # ── 生命周期 ──────────────────────────────

    def start(self) -> None:
        """启动安全监控后台线程。"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._monitor_loop,
            name="SafetyMonitor",
            daemon=True,
        )
        self._thread.start()
        logger.info("安全监控已启动  mode_ch=CH%d  kill_ch=CH%d",
                     self._mode_ch, self._kill_ch)

    def stop(self) -> None:
        """停止安全监控线程。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("安全监控已停止")

    # ── 查询接口 ──────────────────────────────

    def is_guided_allowed(self) -> bool:
        """RC mode_channel 是否允许上位机接管（Guided 模式）。

        Returns:
            True 表示允许上位机控制。
        """
        with self._lock:
            return self._state == SafetyState.GUIDED_ALLOWED

    def is_kill_triggered(self) -> bool:
        """是否已触发紧急停桨。

        Returns:
            True 表示 KILL 已触发，所有上位机控制必须立刻停止。
        """
        with self._lock:
            return self._state == SafetyState.KILL

    def get_safety_state(self) -> SafetyState:
        """获取当前安全状态。

        Returns:
            SafetyState 枚举值。
        """
        with self._lock:
            return self._state

    def register_cancel_token(self, token: CancelToken) -> None:
        """注册一个 CancelToken，安全层可通过它取消当前任务。

        Args:
            token: 任务的取消令牌。
        """
        with self._lock:
            self._cancel_tokens.append(token)
            logger.debug("注册 CancelToken，当前共 %d 个", len(self._cancel_tokens))

    def unregister_cancel_token(self, token: CancelToken) -> None:
        """移除已完成任务的 CancelToken。

        Args:
            token: 待移除的取消令牌。
        """
        with self._lock:
            try:
                self._cancel_tokens.remove(token)
            except ValueError:
                pass

    # ── 内部监控循环 ──────────────────────────

    def _monitor_loop(self) -> None:
        """后台监控主循环——10 Hz 轮询 RC 通道与心跳状态。"""
        while self._running:
            try:
                self._check_safety()
            except Exception as exc:
                logger.error("安全监控异常: %s", exc, exc_info=True)
            time.sleep(self._poll_interval_s)

    def _check_safety(self) -> None:
        """执行一次安全检查。

        优先级：KILL > LINK_LOST > RC 模式切换
        """
        if not self._link.is_connected():
            self._transition_to(SafetyState.LINK_LOST, "飞控未连接")
            return

        vehicle = self._link.get_vehicle()

        # ── 1) 最高优先级：紧急停桨 ────────
        kill_pwm = self._read_rc_channel(vehicle, self._kill_ch)
        if kill_pwm is not None and kill_pwm >= self._kill_pwm_min:
            self._transition_to(SafetyState.KILL, f"紧急停桨触发  CH{self._kill_ch}={kill_pwm}")
            return

        # ── 2) 心跳超时 ──────────────────
        hb_age = self._link.last_heartbeat_age_s()
        if hb_age > self._heartbeat_timeout_s:
            self._transition_to(SafetyState.LINK_LOST, f"心跳超时 ({hb_age:.1f}s)")
            return

        # ── 3) RC 模式通道判断 ───────────
        mode_pwm = self._read_rc_channel(vehicle, self._mode_ch)
        if mode_pwm is not None and mode_pwm >= self._guided_pwm_min:
            new_state = SafetyState.GUIDED_ALLOWED
        else:
            new_state = SafetyState.NORMAL

        self._transition_to(new_state)

    def _transition_to(self, new_state: SafetyState, reason: str = "") -> None:
        """状态转换并执行 failsafe 动作。

        Fix 2：安全事件触发时，不仅取消任务，还执行实际的 failsafe 动作：
            - inhibit_outputs()（激活输出门禁）
            - stop_motion()（停止运动输出）
            - set_mode_nowait()（按策略切模式）
        """
        with self._lock:
            old_state = self._state
            if new_state == old_state:
                return

            self._state = new_state
            if reason:
                logger.warning("安全状态变更: %s -> %s  原因: %s",
                               old_state.value, new_state.value, reason)
            else:
                logger.info("安全状态变更: %s -> %s",
                            old_state.value, new_state.value)

            # ── 新状态进入 GUIDED_ALLOWED：解除门禁 ──
            if new_state == SafetyState.GUIDED_ALLOWED:
                self._release_inhibit()
                return

            # ── 需要执行安全动作的状态转换 ──
            if old_state == SafetyState.GUIDED_ALLOWED:
                # 从 GUIDED_ALLOWED 离开 → 必须执行安全动作
                self._execute_safety_actions(new_state, reason)

            elif new_state in (SafetyState.KILL, SafetyState.LINK_LOST):
                # 非 GUIDED 状态下检测到 KILL/LINK_LOST  → 也要执行
                self._execute_safety_actions(new_state, reason)

    def _execute_safety_actions(self, state: SafetyState, reason: str) -> None:
        """执行安全动作——在 _lock 已持有的环境下调用。

        动作序列：
            1. 激活输出门禁（inhibit gate）
            2. 发送 stop_motion（零速度）
            3. 按 failsafe 策略切模式（KILL 除外）
            4. 取消所有已注册任务
        """
        # 1. 激活输出门禁
        self._activate_inhibit()

        # 2. 发送 stop_motion（在独立线程避免死锁，因为 _lock 已持有）
        try:
            vehicle = self._link.get_vehicle()
            self._emit_stop_motion(vehicle)
        except Exception as exc:
            logger.error("安全动作 stop_motion 失败: %s", exc)

        # 3. 按策略切模式
        if state == SafetyState.KILL:
            # KILL: 不切模式——停桨由飞控自身处理
            # best-effort 发送 force disarm（失败不影响流程）
            try:
                from src.control import force_disarm_nowait
                force_disarm_nowait(vehicle)
            except Exception as exc:
                logger.warning("KILL best-effort force_disarm 失败: %s", exc)
            logger.warning("KILL 状态：上位机已停止所有输出")
        elif state == SafetyState.LINK_LOST:
            # 断链：按配置执行 LAND / LOITER
            self._execute_link_lost_action()
        elif state == SafetyState.NORMAL:
            # 撤销接管：上位机停止输出，飞手接管（不需要切模式）
            logger.info("撤销接管：上位机已停止所有输出，飞手接管")

        # 4. 取消所有任务
        self._cancel_all_tasks(f"安全动作: {state.value} - {reason}")

    def _activate_inhibit(self) -> None:
        """激活 control 层输出门禁。"""
        from src.control import inhibit_outputs
        inhibit_outputs()

    def _release_inhibit(self) -> None:
        """解除 control 层输出门禁。"""
        from src.control import release_outputs
        release_outputs()

    def _emit_stop_motion(self, vehicle: Any) -> None:
        """发送零速度指令停止运动。

        使用 control.stop_motion() 的内部逻辑，但绕过门禁检查。
        """
        from src.control import stop_motion
        stop_motion(vehicle)

    def _execute_link_lost_action(self) -> None:
        """按配置执行断链策略。"""
        try:
            vehicle = self._link.get_vehicle()
            from src.control import set_mode_nowait

            action = self._link_lost_action
            if action == "LAND":
                logger.warning("断链策略执行: LAND")
                set_mode_nowait(vehicle, "LAND")
            elif action == "LOITER":
                logger.warning("断链策略执行: LOITER")
                set_mode_nowait(vehicle, "LOITER")
            else:
                logger.error("未知断链策略: %s，默认 LAND", action)
                set_mode_nowait(vehicle, "LAND")
        except Exception as exc:
            logger.error("断链策略执行失败: %s", exc)

    def _cancel_all_tasks(self, reason: str) -> None:
        """取消所有已注册的任务，并清理已 cancel 的 token。

        注意：此方法在 _lock 已持有的状态下调用。
        """
        for token in self._cancel_tokens:
            if not token.is_cancelled():
                token.cancel(reason)
                logger.warning("已取消任务: %s", reason)
        # Fix D: 清理已 cancel 的 token，防止列表无限增长
        self._cancel_tokens = [
            t for t in self._cancel_tokens if not t.is_cancelled()
        ]

    @staticmethod
    def _read_rc_channel(vehicle: Any, channel: int) -> int | None:
        """安全读取 RC 通道 PWM 值。

        Args:
            vehicle: DroneKit Vehicle 实例。
            channel: 通道号（1-based）。

        Returns:
            PWM 值，读取失败返回 None。
        """
        try:
            channels = vehicle.channels
            if channels and str(channel) in channels:
                return int(channels[str(channel)])
            if channels and channel in channels:
                return int(channels[channel])
        except Exception:
            pass
        return None
