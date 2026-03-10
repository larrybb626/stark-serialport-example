"""
触觉仿生手集成模块 - Revo2 Tactile Grasp Demo (Depth Version)

修改说明：
1. 触觉源改为 Sensor.OutputType.Depth
2. 使用图像处理 (Depth -> Grayscale -> Mean Intensity) 来衡量扰动
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
# 如果没有配置文件，创建一个默认的字典防止报错
if not config_file.exists():
    CONFIG = {
        'collision_detection': {'force_threshold': 50.0, 'smoothing_window': 5}, # 注意：阈值现在是像素值(0-255)
        'tactile': {'target_fps': 30, 'sensor_serial': 'BM000000', 'display_heatmap': True},
        'hand_control': {'grasp_position': 600, 'release_position': 0, 'control_duration': 500},
        'monitoring': {'print_interval': 1.0, 'demo_mode': 'grasp_release'}
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
        self.display_img = None  # 用于可视化的图像
        self.score_history = []  # 用于平滑的历史数据

    def initialize(self):
        """初始化触觉传感器"""
        try:
            logger.info(f"正在连接触觉传感器: {self.sensor_serial}")
            sensor = Sensor.create(
                self.sensor_serial,
                api=CameraSource.AV_V4L2
            )

            if sensor is None:
                logger.error("触觉传感器初始化失败 - Sensor.create() 返回 None")
                self.sensor = None
                return False

            self.sensor = sensor
            logger.info("触觉传感器初始化成功 (Depth Mode)")
            return True
        except Exception as e:
            logger.error(f"触觉传感器初始化异常: {e}")
            self.sensor = None
            return False

    def collect_one_frame(self):
        """
        采集一帧触觉深度数据并处理

        Returns:
            dict: 包含触觉评分的字典
        """
        if not self.sensor:
            return None

        try:
            # 1. 采集深度数据 (Depth)
            # 原始 depth 数据通常是 float 或 int，代表距离/变形量
            raw_depth = self.sensor.selectSensorInfo(
                Sensor.OutputType.Depth
            )

            if raw_depth is None:
                return None

            # 2. 图像处理算法 (User Provided Logic)
            # 将深度数据映射到 0-255 的可视区间
            # clip 限制范围，然后转为 uint8 格式
            depth_vis = np.clip(raw_depth * 200, 0, 255).astype(np.uint8)

            # 3. 计算“扰动值” (Disturbance Score)
            # 使用图像的 平均像素强度 (Mean Intensity) 作为衡量受力大小的标准
            # 当手抓握物体受力时，凝胶变形，像素值总体会升高
            current_score = np.mean(depth_vis)
            max_score = np.max(depth_vis)

            # 保存用于可视化
            self.display_img = depth_vis

            return {
                'raw_depth': raw_depth,
                'depth_vis': depth_vis,
                'force_val': float(current_score), # 这里为了兼容旧接口，key依然叫 force_val
                'max_val': float(max_score),
                'timestamp': time.time()
            }

        except Exception as e:
            logger.error(f"采集触觉数据异常: {e}")
            return None

    def is_collision_detected(self, current_data, threshold=None):
        """
        检测是否发生碰撞/扰动
        """
        if threshold is None:
            threshold = CONFIG['collision_detection']['force_threshold']

        if current_data is None:
            return False

        # 获取当前帧的“受力分数” (即平均深度像素值)
        current_val = current_data['force_val']

        # 更新历史数据用于平滑 (防止单帧噪点导致的误触)
        self.score_history.append(current_val)
        if len(self.score_history) > CONFIG['collision_detection']['smoothing_window']:
            self.score_history.pop(0)

        # 使用移动平均值与阈值比较
        smoothed_val = np.mean(self.score_history)

        return smoothed_val > threshold

    def get_statistics(self):
        """获取统计信息"""
        if self.display_img is None:
            return None
        return {
            'mean': float(np.mean(self.display_img)),
            'max': float(np.max(self.display_img)),
            'std': float(np.std(self.display_img))
        }

    def visualize(self):
        """显示深度图"""
        if not CONFIG['tactile']['display_heatmap'] or self.display_img is None:
            return True

        try:
            # 应用伪彩色，让深度图看起来更直观 (Blue -> Red)
            display_color = cv2.applyColorMap(self.display_img, cv2.COLORMAP_JET)

            # 在图像上显示当前数值
            cv2.putText(display_color, f"Score: {np.mean(self.display_img):.1f}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("Tactile Depth Stream", display_color)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                return False
        except Exception as e:
            logger.debug(f"可视化异常: {e}")

        return True

    def cleanup(self):
        if self.sensor is not None:
            try:
                self.sensor.release()
            except:
                pass
            self.sensor = None
        cv2.destroyAllWindows()


# ==================== 手部控制模块 (保持不变) ====================
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

    async def grasp(self, position=None, duration=None):
        pos = position if position is not None else CONFIG['hand_control']['grasp_position']
        dur = duration if duration is not None else CONFIG['hand_control']['control_duration']
        try:
            await self.client.set_finger_positions_and_durations(self.slave_id, [pos]*6, [dur]*6)
            await asyncio.sleep(dur / 1000.0 + 0.5) # 稍微多等一点时间确保抓稳
        except Exception as e:
            logger.error(f"抓握异常: {e}")

    async def release(self, duration=None):
        pos = CONFIG['hand_control']['release_position']
        dur = duration if duration is not None else CONFIG['hand_control']['control_duration']
        try:
            await self.client.set_finger_positions_and_durations(self.slave_id, [pos]*6, [dur]*6)
            await asyncio.sleep(dur / 1000.0 + 0.2)
        except Exception as e:
            logger.error(f"释放异常: {e}")


# ==================== 主逻辑控制器 ====================
class TactileGraspController:
    def __init__(self, hand_controller, tactile_collector):
        self.hand = hand_controller
        self.tactile = tactile_collector
        self.is_grasping = False

    async def run_grasp_release_demo(self):
        """核心流程：抓握 -> 监测扰动 -> 释放"""
        logger.info(">>> 启动触觉抓握闭环演示")

        try:
            while True:
                # 1. 初始状态：手张开 (确保归位)
                logger.info("准备中...")
                await self.hand.release()
                await asyncio.sleep(1.0)

                # 2. 执行抓握
                logger.info(">>> [1/3] 正在抓握物体...")
                await self.hand.grasp()
                self.is_grasping = True
                logger.info("    抓握完成，建立基准...")

                # 可选：这里可以加一个逻辑，读取抓握后的“基准值”，
                # 然后阈值设为 基准值 + 增量 (自适应阈值)，目前先用固定阈值

                # 3. 监测循环
                logger.info(">>> [2/3] 进入触觉监测模式 (等待扰动)...")
                triggered = await self.monitor_collision_loop()

                # 4. 触发释放
                if triggered:
                    logger.warning(">>> [3/3] 检测到扰动！立即释放！")
                    await self.hand.release()
                    self.is_grasping = False

                logger.info("--- 循环结束，3秒后重试 ---\n")
                await asyncio.sleep(3.0)

        except KeyboardInterrupt:
            logger.info("用户停止演示")
        finally:
            await self.cleanup()

    async def monitor_collision_loop(self):
        """循环检测，直到超过阈值返回 True"""
        frame_count = 0
        last_print = time.time()

        while True:
            # 采集数据
            data = self.tactile.collect_one_frame()
            if data is None:
                await asyncio.sleep(0.01)
                continue

            # 判断碰撞
            # 注意：阈值现在是 0-255 的数值
            # 建议先用 monitor 模式看一下正常抓握时的数值是多少，比如正常抓是40，受力是80，那阈值设60
            is_collision = self.tactile.is_collision_detected(data)

            # 打印状态
            if time.time() - last_print > 1.0:
                stats = self.tactile.get_statistics()
                status = "⚠️ 触发释放" if is_collision else "监测中..."
                logger.info(f"{status} | 当前Score(Mean): {stats['mean']:.1f} | 阈值: {CONFIG['collision_detection']['force_threshold']}")
                last_print = time.time()

            # 可视化
            if not self.tactile.visualize():
                return False

            if is_collision:
                return True

            await asyncio.sleep(0.01)

    async def run_monitor_only(self):
        """仅监测模式：用于调试阈值"""
        logger.info(">>> 启动纯监测模式 (用于校准阈值)")
        logger.info("请观察'当前Score'的变化，以此来设定合适的 --force-threshold")

        while True:
            data = self.tactile.collect_one_frame()
            if data is None: continue

            is_collision = self.tactile.is_collision_detected(data)

            if not self.tactile.visualize(): break

            # 实时打印比较频繁，便于观察数值跳变
            sys.stdout.write(f"\rScore: {data['force_val']:.1f} | Max: {data['max_val']:.0f} | 碰撞: {'YES' if is_collision else 'NO '}   ")
            sys.stdout.flush()

            await asyncio.sleep(0.02)

    async def cleanup(self):
        await self.hand.release()
        self.tactile.cleanup()
        libstark.modbus_close(self.hand.client)


# ==================== 程序入口 ====================
async def main():
    parser = argparse.ArgumentParser(description='Revo2 触觉抓握演示 (深度图版)')
    parser.add_argument('--hand-port', type=str, help='手部串口')
    parser.add_argument('--touch-serial', type=str, default=CONFIG['tactile']['sensor_serial'], help='触觉序列号')
    # 注意：这里的默认阈值改大了，因为像素和之和可能比较大，或者均值在0-255之间
    parser.add_argument('--force-threshold', type=float, default=5.0, help='触发释放的阈值 (0-255之间, 建议先用monitor测一下)')
    parser.add_argument('--mode', type=str, choices=['grasp', 'monitor'], default='grasp', help='grasp=抓握演示, monitor=仅看数据')

    args = parser.parse_args()

    # 更新配置
    CONFIG['collision_detection']['force_threshold'] = args.force_threshold

    # 初始化
    client, slave_id = await open_modbus_revo2(port_name=args.hand_port)
    hand = RevoHandController(client, slave_id)
    await hand.initialize()

    tactile = TactileDataCollector(args.touch_serial)
    if not tactile.initialize():
        sys.exit(1)

    controller = TactileGraspController(hand, tactile)

    try:
        if args.mode == 'monitor':
            await controller.run_monitor_only()
        else:
            await controller.run_grasp_release_demo()
    except Exception as e:
        logger.error(f"运行出错: {e}")
        await controller.cleanup()

if __name__ == "__main__":
    asyncio.run(main())