"""
触觉仿生手集成模块 - Revo2 Tactile Grasp Demo (OK 姿势 + 深度图版)
"""

import asyncio
import cv2
import numpy as np
import time
import sys
import argparse
import yaml
from pathlib import Path

# 动态导入路径
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import logger, libstark

# ==================== 日志简洁化配置 ====================
import logging
import os
class ConciseFormatter(logging.Formatter):
    """自定义格式化器：将绝对路径简化为 ~/ 或 仅文件名"""
    def format(self, record):
        # home = os.path.expanduser("~")
        # 如果路径在用户目录下，替换为 ~
        # if record.pathname.startswith(home):
        #     record.concise_path = record.pathname.replace(home, "~", 1)
        # else:
        #     # 否则只显示最后的文件名
        record.concise_path = os.path.basename(record.pathname)
        return super().format(record)

for handler in logger.handlers:
    # 定义简洁的格式：时间 [等级] [路径:行号] 消息
    new_fmt = '%(asctime)s [%(levelname)s] [%(concise_path)s:%(lineno)d] %(message)s'
    handler.setFormatter(ConciseFormatter(new_fmt))
# ===================================================

try:
    from xensesdk import Sensor
    from xensesdk.xenseInterface.sensorEnum import CameraSource
except ImportError:
    logger.error("触觉SDK (xensesdk) 未安装，请先安装: pip install xensesdk")
    sys.exit(1)

from revo2_utils import open_modbus_revo2


# ==================== 加载配置 ====================
config_file = Path(__file__).parent / "config_needle.yaml"
if not config_file.exists():
    CONFIG = {
        'collision_detection': {'force_threshold': 80.0, 'smoothing_window': 3}, # 窗口稍微调小一点，反应更快
        'tactile': {'target_fps': 30, 'sensor_serial': 'BM000000'},
        'hand_control': {'control_duration': 800},
        'monitoring': {'print_interval': 1.0, 'demo_mode': 'grasp'}
    }
else:
    with open(config_file, 'r', encoding='utf-8') as f:
        CONFIG = yaml.safe_load(f)


# ==================== 触觉数据采集模块 ====================
class TactileDataCollector:
    """触觉数据采集和处理类 (全图去零 ± 阈值版)"""

    def __init__(self, sensor_serial=None):
        self.sensor_serial = sensor_serial
        self.sensor = None
        self.display_img = None
        self.score_history = []
        self.baseline_score = 0.0

    def initialize(self):
        try:
            logger.info(f"正在连接触觉传感器: {self.sensor_serial}")
            sensor = Sensor.create(self.sensor_serial, api=CameraSource.AV_V4L2)
            if sensor is None: return False
            self.sensor = sensor
            return True
        except Exception as e:
            logger.error(f"触觉传感器初始化异常: {e}")
            return False

    def collect_one_frame(self):
        if not self.sensor: return None
        try:
            # 1. 采集深度数据 (Depth)
            raw_depth = self.sensor.selectSensorInfo(Sensor.OutputType.Depth)
            if raw_depth is None: return None

            # 2. 图像处理算法：映射为 0-255 灰度图
            depth_vis = np.clip(raw_depth * 200, 0, 255).astype(np.uint8)

            # 1. 移除 ROI，使用全图
            # 2. 改回 Mean：防止极值达到 255 后无法检测到正向增加的压力
            current_score = np.mean(depth_vis)

            self.display_img = depth_vis

            return {
                'depth_vis': depth_vis,
                'force_val': float(current_score),
                'timestamp': time.time()
            }
        except Exception as e:
            logger.error(f"采集触觉数据异常: {e}")
            return None

    def is_collision_detected(self, current_data, threshold=None):
        if threshold is None:
            threshold = CONFIG['collision_detection']['force_threshold']
        if current_data is None: return False

        current_val = current_data['force_val']

        # 移动平均滤波，防止单帧噪点误触
        self.score_history.append(current_val)
        if len(self.score_history) > CONFIG['collision_detection']['smoothing_window']:
            self.score_history.pop(0)

        smoothed_val = np.mean(self.score_history)

        # 【核心修改】：绝对值增量判断。
        # 等同于 current > base + threshold 或 current < base - threshold
        return abs(smoothed_val - self.baseline_score) > threshold

    def visualize(self):
        """显示深度热力图 (移除绿框)"""
        if self.display_img is None:
            return True

        try:
            # 容错处理：确保传给热力图的是单通道灰度图，防止切换阶段时报错
            gray_img = self.display_img
            if len(gray_img.shape) == 3:
                gray_img = cv2.cvtColor(gray_img, cv2.COLOR_BGR2GRAY)

            # 应用伪彩色，颜色越红代表受力越深
            display_color = cv2.applyColorMap(self.display_img, cv2.COLORMAP_JET)

            if len(self.score_history) > 0:
                score = np.mean(self.score_history)
            else:
                score = 0.0

            # 移除绿框，只保留左上角的文字状态
            score_text = f"Score: {score:.1f} (Base: {self.baseline_score:.1f})"
            cv2.putText(display_color, score_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.imshow("Tactile Depth Stream", display_color)

            # 🌟 安全退出检测：按下 'q' 键时强制清理硬件
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                from common_imports import logger
                logger.warning("\n>>> 🛑 检测到用户退出指令 (Q)，正在强制释放硬件资源...")
                # 显式释放传感器资源，彻底解决下次运行时的 init camera fail 报错
                if hasattr(self, 'cleanup'):
                    self.cleanup()
                return False
        except Exception as e:
            logger.debug(f"可视化异常: {e}")
        return True

    def cleanup(self):
        if self.sensor:
            try: self.sensor.release()
            except: pass
        cv2.destroyAllWindows()


# ==================== 手部控制模块 (修改为 OK 姿势) ====================
class RevoHandController:
    def __init__(self, client, slave_id):
        self.client = client
        self.slave_id = slave_id

    async def initialize(self):
        try:
            await self.client.set_finger_unit_mode(self.slave_id, libstark.FingerUnitMode.Normalized)
            return True
        except Exception as e:
            logger.error(f"手部初始化异常: {e}")
            return False

    async def grasp(self):
        """执行抓握动作 - 定制为 'OK' 姿势 (仅拇指和食指)"""
        dur = CONFIG['hand_control']['control_duration']
        durations = [dur] * 6

        # 假设数组顺序为：[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指侧摆]
        # 食指(600) + 拇指弯曲(600) + 拇指侧摆对向食指(1000)，其余为0
        ok_positions = [600, 700, 450, 0, 0, 0]

        try:
            logger.info(f"手指归位，准备捏合...")
            # 先全部张开作为预备动作
            # await self.client.set_finger_positions_and_durations(self.slave_id, [0]*6, [dur]*6)
            # await asyncio.sleep(dur / 1000.0 + 0.2)

            logger.info(f"执行 OK 抓握姿势: {ok_positions}")
            await self.client.set_finger_positions_and_durations(self.slave_id, ok_positions, durations)
            await asyncio.sleep(dur / 1000.0 + 0.5)
        except Exception as e:
            logger.error(f"抓握异常: {e}")

    async def release(self):
        """释放所有手指"""
        dur = CONFIG['hand_control']['control_duration']
        durations = [dur] * 6
        release_positions = [0, 0, 0, 0, 0, 0]

        try:
            logger.info("执行释放动作 (全开)")
            await self.client.set_finger_positions_and_durations(self.slave_id, release_positions, durations)
            await asyncio.sleep(dur / 1000.0 + 0.2)
        except Exception as e:
            logger.error(f"释放异常: {e}")


# ==================== 主逻辑控制器 ====================
class TactileGraspController:
    def __init__(self, hand_controller, tactile_collector):
        self.hand = hand_controller
        self.tactile = tactile_collector

    async def run_grasp_release_demo(self):
        logger.info(">>> 启动触觉抓握闭环演示")
        try:
            while True:
                # await self.hand.release()
                await asyncio.sleep(1.0)

                # 1. 执行OK抓握
                logger.info(">>> [1/3] 正在用 OK 手势抓握物体...")
                await self.hand.grasp()

                # 2. 监测循环前的基准校准 (去零操作)
                logger.info(">>> [2/3] 进入触觉监测模式，开始 5s 稳定期并提取基准压力值(去零)...")

                calibration_scores = []
                calib_start = time.time()

                # 动态等待2秒，期间持续采集数据和刷新画面
                t_wait = 5.0
                while time.time() - calib_start < t_wait:
                    data = self.tactile.collect_one_frame()
                    if data is not None:
                        calibration_scores.append(data['force_val'])
                        self.tactile.visualize()  # 保持画面不卡顿
                    await asyncio.sleep(0.01)

                # 计算基准值：截取后半段（后1秒）的数据求平均，避开刚闭合时的微震荡
                if len(calibration_scores) > 10:
                    stable_scores = calibration_scores[len(calibration_scores)//2:]
                    self.tactile.baseline_score = sum(stable_scores) / len(stable_scores)
                else:
                    self.tactile.baseline_score = 0.0

                self.tactile.score_history.clear()

                th = CONFIG['collision_detection']['force_threshold']

                logger.info(f">>> 稳定期结束！当前静止基准压力值设为: {self.tactile.baseline_score:.1f}")
                logger.info(f">>> 正式进入扰动监测状态 (触发条件: 偏离基准值超过 ±{th:.1f})...")

                triggered = await self.monitor_collision_loop()

                # 3. 触发释放
                if triggered:
                    logger.warning(">>> [3/3] ⚠️ 检测到强烈扰动！自动释放！")
                    # await self.hand.release()
                    # 释放后清理基准值
                    self.tactile.baseline_score = 0.0

                logger.info("--- 循环结束，3秒后重试 ---\n")
                await asyncio.sleep(3.0)
        except KeyboardInterrupt:
            pass
        finally:
            await self.cleanup()

    async def monitor_collision_loop(self):
        last_print = time.time()
        th = CONFIG['collision_detection']['force_threshold']

        while True:
            data = self.tactile.collect_one_frame()
            if data is None:
                await asyncio.sleep(0.01)
                continue

            is_collision = self.tactile.is_collision_detected(data)

            # 每秒打印一次日志，避免刷屏
            if time.time() - last_print > 1.0:
                status = "⚠️ 触发释放" if is_collision else "监测中..."
                upper_bound = self.tactile.baseline_score + th
                lower_bound = max(0, self.tactile.baseline_score - th) # 下限不低于0

                logger.info(f"{status} | 当前: {data['force_val']:.1f} | 基准: {self.tactile.baseline_score:.1f} | 安全区间: [{lower_bound:.1f}, {upper_bound:.1f}]")
                last_print = time.time()

            # 实时渲染图像
            if not self.tactile.visualize():
                return False

            if is_collision:
                return True

            await asyncio.sleep(0.01)

    async def run_monitor_only(self):
        logger.info(">>> 启动纯监测模式 (请观察屏幕图像和数值以校准阈值)")
        while True:
            data = self.tactile.collect_one_frame()
            if data is None: continue

            is_collision = self.tactile.is_collision_detected(data)
            if not self.tactile.visualize(): break

            sys.stdout.write(f"\r当前 全图平均 Score: {data['force_val']:.1f} | 碰撞状态: {'已超阈值' if is_collision else '安全'}   ")
            sys.stdout.flush()
            await asyncio.sleep(0.02)

    async def cleanup(self):
        # 加入异常捕获，防止多次 cleanup 导致 Modbus 报错
        # try:
        #     await self.hand.release()
        # except:
        #     pass
        self.tactile.cleanup()
        try:
            libstark.modbus_close(self.hand.client)
        except:
            pass


# ==================== 程序入口 ====================
async def main():
    parser = argparse.ArgumentParser(description='Revo2 触觉抓握演示 (OK 姿势)')
    parser.add_argument('--hand-port', type=str, help='手部串口')
    parser.add_argument('--touch-serial', type=str, default=CONFIG['tactile']['sensor_serial'], help='触觉序列号')
    # 因为现在取的是 Max 最大值，最高可以是 255，建议默认阈值设高一些
    parser.add_argument('--force-threshold', type=float, default=CONFIG['collision_detection']['force_threshold'], help='触发释放的阈值 (最大像素强度 0-255)')
    parser.add_argument('--mode', type=str, choices=['grasp', 'monitor'], default='grasp', help='grasp=闭环演示, monitor=仅监测用于定阈值')

    args = parser.parse_args()
    # CONFIG['collision_detection']['force_threshold'] = args.force_threshold

    client, slave_id = await open_modbus_revo2(port_name=args.hand_port)
    hand = RevoHandController(client, slave_id)
    await hand.initialize()

    tactile = TactileDataCollector(args.touch_serial)
    if not tactile.initialize(): sys.exit(1)

    controller = TactileGraspController(hand, tactile)
    try:
        if args.mode == 'monitor': await controller.run_monitor_only()
        else: await controller.run_grasp_release_demo()
    except Exception as e:
        logger.error(f"运行出错: {e}")
        await controller.cleanup()

if __name__ == "__main__":
    asyncio.run(main())