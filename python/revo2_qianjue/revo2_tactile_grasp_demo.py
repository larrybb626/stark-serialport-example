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

try:
    from xensesdk import Sensor
    from xensesdk.xenseInterface.sensorEnum import CameraSource
except ImportError:
    logger.error("触觉SDK (xensesdk) 未安装，请先安装: pip install xensesdk")
    sys.exit(1)

from revo2_utils import open_modbus_revo2


# ==================== 加载配置 ====================
config_file = Path(__file__).parent / "config.yaml"
if not config_file.exists():
    CONFIG = {
        'collision_detection': {'force_threshold': 60.0, 'smoothing_window': 5},
        'tactile': {'target_fps': 30, 'sensor_serial': 'BM000000'},
        'hand_control': {'control_duration': 800},
        'monitoring': {'print_interval': 1.0, 'demo_mode': 'grasp'}
    }
else:
    with open(config_file, 'r', encoding='utf-8') as f:
        CONFIG = yaml.safe_load(f)


# ==================== 触觉数据采集模块 ====================
class TactileDataCollector:
    """触觉数据采集和处理类 (深度图版)"""

    def __init__(self, sensor_serial=None):
        self.sensor_serial = sensor_serial
        self.sensor = None
        self.display_img = None
        self.score_history = []

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

            # 3. 计算“扰动值” (Score = 平均形变体积)
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
        return smoothed_val > threshold

    def visualize(self):
        """强制显示深度热力图"""
        if self.display_img is None:
            return True
        try:
            # 应用伪彩色，颜色越红代表受力越深
            display_color = cv2.applyColorMap(self.display_img, cv2.COLORMAP_JET)

            # 实时显示 Score
            score_text = f"Score(Mean): {np.mean(self.display_img):.1f}"
            cv2.putText(display_color, score_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.imshow("Tactile Depth Stream", display_color)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
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
        ok_positions = [450, 800, 500, 0, 0, 0]

        try:
            logger.info(f"手指归位，准备捏合...")
            # 先全部张开作为预备动作
            await self.client.set_finger_positions_and_durations(self.slave_id, [0]*6, [dur]*6)
            await asyncio.sleep(dur / 1000.0 + 0.2)

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
                await self.hand.release()
                await asyncio.sleep(1.0)

                # 1. 执行OK抓握
                logger.info(">>> [1/3] 正在用 OK 手势抓握物体...")
                await self.hand.grasp()

                # 2. 监测循环
                logger.info(">>> [2/3] 进入触觉监测模式，等待 2s 以稳定触觉信号...")

                await asyncio.sleep(2.0)
                self.tactile.score_history.clear()
                logger.info(">>> 稳定期结束，正式进入扰动监测状态...")

                triggered = await self.monitor_collision_loop()

                # 3. 触发释放
                if triggered:
                    logger.warning(">>> [3/3] ⚠️ 检测到强烈扰动！自动释放！")
                    await self.hand.release()

                logger.info("--- 循环结束，3秒后重试 ---\n")
                await asyncio.sleep(3.0)
        except KeyboardInterrupt:
            pass
        finally:
            await self.cleanup()

    async def monitor_collision_loop(self):
        last_print = time.time()
        while True:
            data = self.tactile.collect_one_frame()
            if data is None:
                await asyncio.sleep(0.01)
                continue

            is_collision = self.tactile.is_collision_detected(data)

            # 每秒打印一次日志，避免刷屏
            if time.time() - last_print > 1.0:
                status = "⚠️ 触发释放" if is_collision else "监测中..."
                logger.info(f"{status} | 当前Score(Mean): {data['force_val']:.1f} | 阈值: {CONFIG['collision_detection']['force_threshold']}")
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

            sys.stdout.write(f"\r当前形变 Score: {data['force_val']:.1f} | 碰撞状态: {'已超阈值' if is_collision else '安全'}   ")
            sys.stdout.flush()
            await asyncio.sleep(0.02)

    async def cleanup(self):
        await self.hand.release()
        self.tactile.cleanup()
        libstark.modbus_close(self.hand.client)


# ==================== 程序入口 ====================
async def main():
    parser = argparse.ArgumentParser(description='Revo2 触觉抓握演示 (OK 姿势)')
    parser.add_argument('--hand-port', type=str, help='手部串口')
    parser.add_argument('--touch-serial', type=str, default=CONFIG['tactile']['sensor_serial'], help='触觉序列号')
    parser.add_argument('--force-threshold', type=float, default=5.0, help='触发释放的阈值 (平均像素强度)')
    parser.add_argument('--mode', type=str, choices=['grasp', 'monitor'], default='grasp', help='grasp=闭环演示, monitor=仅监测用于定阈值')

    args = parser.parse_args()
    CONFIG['collision_detection']['force_threshold'] = args.force_threshold

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