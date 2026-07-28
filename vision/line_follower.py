#!/usr/bin/env python3
"""
RDK X5 视觉决策模块

功能：
  - 黑线巡线（Otsu 二值化 + 轮廓质心跟踪）
  - 红色锥桶避障（HSV 色彩分割 + 形态学滤波）
  - 蓝色方块抓取（色块检测 + 时序状态机）
  - 红线计数（地面标记检测）

架构：
  VisionPipeline (视觉流水线)
    ├─ LineDetector     : 巡线检测器
    ├─ ConeDetector     : 锥桶避障检测器
    ├─ BlockDetector    : 方块抓取检测器
    └─ RedLineDetector  : 红线计数检测器

  CarController (小车控制器)
    ├─ StateMachine     : 主状态机 (FOLLOW / AVOID / PICK)
    └─ SerialBridge     : 串口通信桥接
"""

import time
import logging
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

import cv2
import numpy as np
import serial
import serial.serialutil

# ==================== 日志配置 ====================
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('RC-Car')

# ==================== 配置数据类 ====================

@dataclass
class VisionConfig:
    """视觉检测参数集中配置"""
    # 通用
    camera_index: int = 0
    view_crop_ratio: float = 0.7
    roi_ratio: float = 0.5

    # 巡线
    line_min_area: int = 300
    line_slight_turn_thresh: float = 0.02
    line_hard_turn_thresh: float = 0.08
    line_ema_alpha: float = 0.6

    # 锥桶避障
    cone_min_area: int = 200
    cone_min_height_px: int = 3
    cone_hsv_lower1: Tuple = (0, 60, 40)
    cone_hsv_upper1: Tuple = (14, 255, 255)
    cone_hsv_lower2: Tuple = (166, 60, 40)
    cone_hsv_upper2: Tuple = (180, 255, 255)

    # 蓝色方块
    block_min_area: int = 3000
    block_min_side_px: int = 50
    block_hsv_lower: Tuple = (100, 120, 50)
    block_hsv_upper: Tuple = (140, 255, 255)

    # 红线
    redline_min_area: int = 800
    redline_min_aspect: float = 3.0
    redline_cooldown_sec: float = 125.0


@dataclass
class ControlConfig:
    """控制时序参数"""
    # 避障序列
    avoid_stop_sec: float = 1.2
    avoid_back_sec: float = 1.1
    avoid_right_sec: float = 1.0
    avoid_forward_sec: float = 2.0
    avoid_left_sec: float = 1.8
    avoid_forward2_sec: float = 2.3
    avoid_cooldown_sec: float = 0.8

    # 蓝色方块动作序列
    blue_stop_sec: float = 0.8
    blue_open_sec: float = 2.0
    blue_turn_sec: float = 2.0
    blue_grab_sec: float = 2.0
    blue_return_sec: float = 2.0
    blue_cooldown_sec: float = 0.8


@dataclass
class SerialConfig:
    """串口配置"""
    port: str = '/dev/ttyS1'
    baud: int = 115200


# ==================== 视觉检测器 ====================

class LineDetector:
    """黑线巡线检测器"""

    def __init__(self, config: VisionConfig):
        self.config = config
        self._last_nx: Optional[float] = None

    def detect(self, frame: np.ndarray) -> Optional[str]:
        """
        分析 ROI 区域的黑线位置，返回方向命令。
        """
        h, w = frame.shape[:2]
        roi_y = int(h * self.config.roi_ratio)
        roi = frame[roi_y:, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255,
                                  cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours
                    if cv2.contourArea(c) >= self.config.line_min_area]

        if not contours:
            return 'STOP'

        max_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(max_contour)
        if M['m00'] == 0:
            return 'STOP'

        cx = int(M['m10'] / M['m00'])
        nx = cx / w

        # EMA 平滑
        if self._last_nx is not None:
            nx = self.config.line_ema_alpha * nx + (1 - self.config.line_ema_alpha) * self._last_nx
        self._last_nx = nx

        delta = nx - 0.5
        cfg = self.config
        if delta > cfg.line_hard_turn_thresh:
            return 'RIGHT'
        elif delta < -cfg.line_hard_turn_thresh:
            return 'LEFT'
        elif delta > cfg.line_slight_turn_thresh:
            return 'LITTLERIGHT'
        elif delta < -cfg.line_slight_turn_thresh:
            return 'LITTLELEFT'
        return 'FORWARD'

    def reset(self):
        self._last_nx = None


class ConeDetector:
    """红色锥桶避障检测器"""

    def __init__(self, config: VisionConfig):
        self.config = config

    def detect(self, frame: np.ndarray) -> Tuple[bool, int]:
        """
        返回 (是否检测到, 最大锥桶像素高度)。
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cfg = self.config

        mask1 = cv2.inRange(hsv, np.array(cfg.cone_hsv_lower1),
                                    np.array(cfg.cone_hsv_upper1))
        mask2 = cv2.inRange(hsv, np.array(cfg.cone_hsv_lower2),
                                    np.array(cfg.cone_hsv_upper2))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.GaussianBlur(mask, (3, 3), 0)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        max_h = 0
        for c in contours:
            if cv2.contourArea(c) < cfg.cone_min_area:
                continue

            x, y, w, h = cv2.boundingRect(c)
            if h / max(w, 1) < 0.7:
                continue

            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = cv2.contourArea(c) / hull_area if hull_area > 0 else 0

            if solidity > 0.6 and h > max_h:
                max_h = h

        return max_h > 0, max_h


class BlockDetector:
    """蓝色方块检测器"""

    def __init__(self, config: VisionConfig):
        self.config = config

    def detect(self, frame: np.ndarray) -> bool:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cfg = self.config

        mask = cv2.inRange(hsv, np.array(cfg.block_hsv_lower),
                                     np.array(cfg.block_hsv_upper))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)
            if area < cfg.block_min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w < cfg.block_min_side_px or h < cfg.block_min_side_px:
                continue
            ratio = w / max(h, 1)
            if 0.8 <= ratio <= 1.25:
                return True
        return False


class RedLineDetector:
    """地面红线计数检测器"""

    def __init__(self, config: VisionConfig):
        self.config = config
        self._count = 0
        self._last_seen: float = 0.0

    def detect(self, frame: np.ndarray) -> bool:
        """检测红线并自动计数（带冷却时间）"""
        h, w = frame.shape[:2]
        roi = frame[int(h * 0.6):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, np.array((0, 70, 60)), np.array((12, 255, 255)))
        mask2 = cv2.inRange(hsv, np.array((168, 70, 60)), np.array((180, 255, 255)))
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        for c in contours:
            if cv2.contourArea(c) < self.config.redline_min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            aspect = bw / max(bh, 1)
            if aspect >= self.config.redline_min_aspect and bw > w * 0.3:
                detected = True
                break

        now = time.time()
        if detected and now - self._last_seen > self.config.redline_cooldown_sec:
            self._count += 1
            self._last_seen = now

        return detected

    @property
    def count(self) -> int:
        return self._count

    def reset(self):
        self._count = 0
        self._last_seen = 0.0


# ==================== 串口桥接 ====================

class SerialBridge:
    """STM32 串口通信桥接"""

    def __init__(self, config: SerialConfig):
        self.config = config
        self._ser: Optional[serial.Serial] = None
        self._last_cmd: Optional[str] = None

    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(
                self.config.port, self.config.baud, timeout=0.1
            )
            logger.info(f'Serial connected: {self.config.port} @ {self.config.baud}')
            return True
        except serial.serialutil.SerialException as e:
            logger.error(f'Serial connection failed: {e}')
            return False

    def send(self, cmd: str):
        """发送命令（自动去重：相同命令不重复发送）"""
        if not cmd or cmd == self._last_cmd:
            return
        if self._ser and self._ser.is_open:
            payload = f'@{cmd}\r\n'
            self._ser.write(payload.encode('utf-8'))
            self._last_cmd = cmd
            logger.debug(f'TX: {cmd}')

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            logger.info('Serial closed')


# ==================== 状态机 ====================

class CarState(Enum):
    FOLLOW = auto()       # 巡线模式
    AVOID = auto()        # 避障序列
    PICK_ACTION = auto()  # 蓝色方块抓取序列
    RED_LINE_STOP = auto() # 红线停车


class StateMachine:
    """主控制状态机"""

    def __init__(self, ctrl_cfg: ControlConfig):
        self.ctrl_cfg = ctrl_cfg
        self.state = CarState.FOLLOW
        self._avoid_t0: float = 0.0
        self._cooldown_until: float = 0.0
        self._blue_stage: int = 0
        self._blue_t0: float = 0.0
        self._blue_sent: bool = False
        self._blue_cooldown: float = 0.0
        self._redline_open_sent: bool = False

    @property
def avoid_sequence(self) -> List[Tuple[str, float]]:
        cfg = self.ctrl_cfg
        return [
            ('AVOID_STOP',    cfg.avoid_stop_sec),
            ('AVOID_BACK',    cfg.avoid_back_sec),
            ('AVOID_RIGHT',   cfg.avoid_right_sec),
            ('AVOID_FORWARD', cfg.avoid_forward_sec),
            ('AVOID_LEFT',    cfg.avoid_left_sec),
            ('AVOID_FORWARD', cfg.avoid_forward2_sec),
        ]

    @property
    def blue_sequence(self) -> List[Tuple[str, float]]:
        cfg = self.ctrl_cfg
        return [
            ('STOP',        cfg.blue_stop_sec),
            ('BLUE_OPEN',   cfg.blue_open_sec),
            ('BLUE_TURN',   cfg.blue_turn_sec),
            ('BLUE_GRAB',   cfg.blue_grab_sec),
            ('BLUE_RETURN', cfg.blue_return_sec),
        ]

    def trigger_avoid(self):
        self.state = CarState.AVOID
        self._avoid_t0 = time.time()

    def trigger_blue_pick(self):
        self.state = CarState.PICK_ACTION
        self._blue_stage = 0
        self._blue_t0 = time.time()
        self._blue_sent = False

    def trigger_redline_stop(self):
        self.state = CarState.RED_LINE_STOP
        self._redline_open_sent = False

    def update(self) -> Optional[str]:
        """状态机更新，返回要发送的命令（或 None）"""
        now = time.time()

        if self.state == CarState.AVOID:
            return self._update_avoid(now)
        elif self.state == CarState.PICK_ACTION:
            return self._update_blue(now)
        elif self.state == CarState.RED_LINE_STOP:
            return self._update_redline()

        return None  # FOLLOW 模式由外部处理

    def _update_avoid(self, now: float) -> Optional[str]:
        elapsed = now - self._avoid_t0
        total = 0.0
        for cmd, duration in self.avoid_sequence:
            total += duration
            if elapsed < total:
                return cmd
        # 避障序列完成
        self.state = CarState.FOLLOW
        self._cooldown_until = now + self.ctrl_cfg.avoid_cooldown_sec
        return 'FORWARD'

    def _update_blue(self, now: float) -> Optional[str]:
        if self._blue_stage >= len(self.blue_sequence):
            self.state = CarState.FOLLOW
            self._blue_cooldown = now + self.ctrl_cfg.blue_cooldown_sec
            return 'FORWARD'

        cmd, duration = self.blue_sequence[self._blue_stage]
        if not self._blue_sent:
            self._blue_sent = True
            return cmd

        if now - self._blue_t0 >= duration:
            self._blue_stage += 1
            self._blue_t0 = now
            self._blue_sent = False

        return None

    def _update_redline(self) -> Optional[str]:
        if not self._redline_open_sent:
            self._redline_open_sent = True
            return 'BLUE_OPEN'
        return 'STOP'

    @property
    def is_following(self) -> bool:
        return self.state == CarState.FOLLOW

    @property
    def cooldown_active(self) -> bool:
        return time.time() < self._cooldown_until or time.time() < self._blue_cooldown


# ==================== 视觉流水线 ====================

class VisionPipeline:
    """视觉检测流水线"""

    def __init__(self, config: VisionConfig):
        self.config = config
        self.line = LineDetector(config)
        self.cone = ConeDetector(config)
        self.block = BlockDetector(config)
        self.redline = RedLineDetector(config)

    @staticmethod
    def center_crop(frame: np.ndarray, ratio: float) -> np.ndarray:
        if ratio >= 0.999:
            return frame
        h, w = frame.shape[:2]
        ch, cw = max(1, int(h * ratio)), max(1, int(w * ratio))
        y0, x0 = (h - ch) // 2, (w - cw) // 2
        return frame[y0:y0 + ch, x0:x0 + cw]


# ==================== 主控制器 ====================

class CarController:
    """小车主控制器"""

    def __init__(self,
                 vision_cfg: VisionConfig,
                 ctrl_cfg: ControlConfig,
                 serial_cfg: SerialConfig):
        self.vision = VisionPipeline(vision_cfg)
        self.serial = SerialBridge(serial_cfg)
        self.fsm = StateMachine(ctrl_cfg)
        self._cap = None

    def start(self) -> bool:
        self._cap = cv2.VideoCapture(self.vision.config.camera_index)
        if not self._cap.isOpened():
            logger.error(f'Camera open failed: index={self.vision.config.camera_index}')
            return False
        self.serial.connect()
        logger.info('Car controller started')
        return True

    def run(self):
        """主循环"""
        if not self.start():
            return

        try:
            while True:
                ret, frame = self._cap.read()
                if not ret:
                    logger.warning('Frame read failed')
                    time.sleep(0.05)
                    continue

                frame = self.vision.center_crop(frame, self.vision.config.view_crop_ratio)

                # 状态机非 FOLLOW 状态：执行时序序列
                if not self.fsm.is_following:
                    cmd = self.fsm.update()
                    if cmd:
                        self.serial.send(cmd)
                    time.sleep(0.03)
                    continue

                # FOLLOW 状态：视觉决策
                cmd = self._decide(frame)
                if cmd:
                    self.serial.send(cmd)
                time.sleep(0.03)

        except KeyboardInterrupt:
            logger.info('Interrupted by user')
        finally:
            self.stop()

    def stop(self):
        self.serial.close()
        if self._cap:
            self._cap.release()
        logger.info('Car controller stopped')

    def _decide(self, frame: np.ndarray) -> Optional[str]:
        """FOLLOW 状态下的视觉决策"""
        cfg = self.vision.config
        fsm = self.fsm

        # 红线检测
        if self.vision.redline.detect(frame):
            if self.vision.redline.count >= 2:
                fsm.trigger_redline_stop()
                return fsm.update()

        # 蓝色方块检测
        if not fsm.cooldown_active and self.vision.block.detect(frame):
            fsm.trigger_blue_pick()
            return fsm.update()

        # 红色锥桶避障
        detected, cone_h = self.vision.cone.detect(frame)
        if detected and cone_h >= cfg.cone_min_height_px and not fsm.cooldown_active:
            fsm.trigger_avoid()
            return 'AVOID_STOP'

        # 巡线
        return self.vision.line.detect(frame)


# ==================== 入口 ====================

def main():
    vision_cfg = VisionConfig()
    ctrl_cfg = ControlConfig()
    serial_cfg = SerialConfig()

    controller = CarController(vision_cfg, ctrl_cfg, serial_cfg)
    controller.run()


if __name__ == '__main__':
    main()
