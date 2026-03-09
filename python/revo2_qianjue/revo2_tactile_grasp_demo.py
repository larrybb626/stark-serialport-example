"""
触觉仿生手集成模块 - Revo2 Tactile Grasp Demo

本模块实现了一个完整的触觉仿生手控制系统，包含三个主要部分：

1. 数据解码部分：
   - 同步采集触觉信号（使用触觉模块SDK）
   - 主要使用 Sensor.OutputType.Force3DTuned
   - 计算触觉力的合力模值

2. 算法计算-阈值判断：
   - 监测触觉力是否超过设定阈值
   - 当超过阈值时，表示物体受到扰动或发生碰撞
   - 返回布尔变量指示碰撞状态

3. 实时控制：
   - 手指先摆到抓握姿势
   - 进入循环监测触觉信号
   - 当检测到超过阈值的力，自动释放手指

工作流程：
- 手先摆到抓握姿势（握住物体）
- 物体受到扰动
- 触觉模块识别到超过阈值
- 手自动放开

使用方式：
    python revo2_tactile_grasp_demo.py [--hand-port PORT] [--touch-serial SERIAL] [--force-threshold THRESHOLD]

参数说明：
    --hand-port PORT: 手的串口号（如 /dev/ttyUSB0），默认自动检测
    --touch-serial SERIAL: 触觉传感器序列号（如 BM000026），默认自动检测
    --force-threshold THRESHOLD: 触觉力阈值（牛顿），默认 5.0
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
    logger.error(f"配置文件不存在: {config_file}")
    sys.exit(1)

with open(config_file, 'r', encoding='utf-8') as f:
    CONFIG = yaml.safe_load(f)


# ==================== 触觉数据采集模块 ====================
class TactileDataCollector:
    """触觉数据采集和处理类"""
    
    def __init__(self, sensor_serial=None):
        """
        初始化触觉数据采集器
        
        Args:
            sensor_serial (str, optional): 触觉传感器序列号
        """
        self.sensor_serial = sensor_serial
        self.sensor = None
        self.force_norm = None  # 保留用于 visualize 和 get_force_statistics
        self.force_history = []
        
    def initialize(self):
        """初始化触觉传感器"""
        try:
            logger.info(f"正在连接触觉传感器: {self.sensor_serial}")
            sensor = Sensor.create(
                self.sensor_serial, 
                api=CameraSource.AV_V4L2
            )
            
            if sensor is None:
                logger.error("触觉传感器初始化失败 - Sensor.create() 返回 None，请检查连接和序列号")
                self.sensor = None
                return False
            
            self.sensor = sensor
            logger.info("触觉传感器初始化成功")
            return True
        except Exception as e:
            logger.error(f"触觉传感器初始化异常: {e}")
            self.sensor = None
            return False
    
    def collect_one_frame(self):
        """
        采集一帧触觉数据
        
        Returns:
            dict: 包含触觉数据的字典，包括 force_3d_tuned, force_norm 等
        """
        if not self.sensor:
            return None
        
        try:
            # 采集 Force3DTuned 数据
            force_3d_tuned = self.sensor.selectSensorInfo(
                Sensor.OutputType.Force3DTuned
            )
            
            if force_3d_tuned is None:
                return None
            
            # 计算触觉力的合力（矩阵的模长）
            # force_3d_tuned 的形状为 (height, width, 3)
            force_norm = np.linalg.norm(force_3d_tuned, axis=2)  # (height, width)
            
            # 取最大值作为该帧的触觉强度
            max_force = np.max(force_norm)
            mean_force = np.mean(force_norm)
            
            # 保存 force_norm 用于可视化
            self.force_norm = force_norm
            
            return {
                'force_3d_tuned': force_3d_tuned,
                'force_norm': force_norm,
                'max_force': float(max_force),
                'mean_force': float(mean_force),
                'timestamp': time.time()
            }
        
        except Exception as e:
            logger.error(f"采集触觉数据异常: {e}")
            return None
    
    def is_collision_detected(self, current_data, threshold=None):
        """
        检测是否发生碰撞（触觉力超过阈值）
        
        Args:
            current_data (dict): 当前帧的触觉数据
            threshold (float, optional): 阈值，如果为None则使用全局配置
            
        Returns:
            bool: 是否检测到碰撞
        """
        if threshold is None:
            threshold = CONFIG['collision_detection']['force_threshold']
        
        if current_data is None:
            return False
        
        max_force = current_data['max_force']
        
        # 更新历史数据用于平滑
        self.force_history.append(max_force)
        if len(self.force_history) > CONFIG['collision_detection']['smoothing_window']:
            self.force_history.pop(0)
        
        # 使用移动平均来平滑噪声
        smoothed_force = np.mean(self.force_history)
        
        return smoothed_force > threshold
    
    def get_force_statistics(self):
        """获取触觉力统计信息"""
        if not self.force_norm:
            return None
        
        return {
            'max': float(np.max(self.force_norm)),
            'mean': float(np.mean(self.force_norm)),
            'std': float(np.std(self.force_norm))
        }
    
    def visualize(self):
        """显示触觉力热力图（可选）"""
        if not CONFIG['tactile']['display_heatmap'] or self.force_norm is None:
            return
        
        try:
            # 归一化显示
            display = (self.force_norm / np.max(self.force_norm) * 255).astype(np.uint8)
            display_color = cv2.applyColorMap(display, cv2.COLORMAP_JET)
            
            cv2.imshow("Tactile Force Heatmap", display_color)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                return False
        except Exception as e:
            logger.debug(f"可视化异常: {e}")
        
        return True
    
    def cleanup(self):
        """清理资源"""
        if self.sensor is not None:
            try:
                self.sensor.release()
                logger.info("触觉传感器资源已释放")
            except Exception as e:
                logger.debug(f"释放触觉传感器异常: {e}")
            finally:
                self.sensor = None
        
        try:
            cv2.destroyAllWindows()
        except:
            pass


# ==================== 手部控制模块 ====================
class RevoHandController:
    """Revo2手部控制类"""
    
    def __init__(self, client, slave_id):
        """
        初始化手部控制器
        
        Args:
            client: Modbus客户端
            slave_id: 设备ID
        """
        self.client = client
        self.slave_id = slave_id
        self.current_positions = [0] * 6
        
    async def initialize(self):
        """初始化手部配置"""
        try:
            # 设置为归一化模式
            await self.client.set_finger_unit_mode(
                self.slave_id, 
                libstark.FingerUnitMode.Normalized
            )
            logger.info("手部控制模式已设置为归一化模式")
            return True
        except Exception as e:
            logger.error(f"手部初始化异常: {e}")
            return False
    
    async def grasp(self, position=None, duration=None):
        """
        执行抓握动作
        
        Args:
            position (int, optional): 目标位置（0-1000），默认使用配置值
            duration (int, optional): 持续时间（毫秒），默认使用配置值
        """
        if position is None:
            position = CONFIG['hand_control']['grasp_position']
        if duration is None:
            duration = CONFIG['hand_control']['control_duration']
        
        try:
            positions = [position] * 6
            durations = [duration] * 6
            
            logger.info(f"执行抓握动作: 位置={position}, 时间={duration}ms")
            await self.client.set_finger_positions_and_durations(
                self.slave_id, positions, durations
            )
            self.current_positions = positions
            
            # 等待手指到达目标位置
            await asyncio.sleep(duration / 1000.0 + 0.2)
            
        except Exception as e:
            logger.error(f"抓握动作执行异常: {e}")
    
    async def release(self, duration=None):
        """
        执行释放动作
        
        Args:
            duration (int, optional): 持续时间（毫秒），默认使用配置值
        """
        if duration is None:
            duration = CONFIG['hand_control']['control_duration']
        
        try:
            positions = [CONFIG['hand_control']['release_position']] * 6
            durations = [duration] * 6
            
            logger.info(f"执行释放动作: 时间={duration}ms")
            await self.client.set_finger_positions_and_durations(
                self.slave_id, positions, durations
            )
            self.current_positions = positions
            
            # 等待手指释放
            await asyncio.sleep(duration / 1000.0 + 0.2)
            
        except Exception as e:
            logger.error(f"释放动作执行异常: {e}")
    
    async def get_motor_status(self):
        """获取马达状态"""
        try:
            status = await self.client.get_motor_status(self.slave_id)
            return status
        except Exception as e:
            logger.error(f"获取马达状态异常: {e}")
            return None


# ==================== 主控制循环 ====================
class TactileGraspController:
    """触觉抓握控制器 - 整合触觉和手控制"""
    
    def __init__(self, hand_controller, tactile_collector):
        """
        初始化触觉抓握控制器
        
        Args:
            hand_controller (RevoHandController): 手部控制器
            tactile_collector (TactileDataCollector): 触觉数据采集器
        """
        self.hand = hand_controller
        self.tactile = tactile_collector
        self.is_grasping = False
        self.collision_detected = False
        self.stats = {
            'frame_count': 0,
            'collision_count': 0,
            'start_time': time.time()
        }
        
    async def run_grasp_release_demo(self):
        """
        运行抓握-释放演示
        
        流程：
        1. 手执行抓握动作
        2. 持续监测触觉信号
        3. 当检测到超过阈值的触觉力时，自动释放
        4. 重复循环
        """
        logger.info("=" * 60)
        logger.info("启动触觉抓握-释放演示")
        logger.info("=" * 60)
        
        try:
            while True:
                # 1. 抓握
                logger.info("\n[步骤1] 执行抓握...")
                await self.hand.grasp()
                self.is_grasping = True
                logger.info("✓ 抓握完成，开始监测触觉信号...")
                
                # 2. 监测阶段
                await self.monitor_and_release()
                
                # 3. 等待后重新开始
                logger.info("\n等待3秒后重新开始循环...\n")
                await asyncio.sleep(3.0)
                
        except KeyboardInterrupt:
            logger.info("\n用户中断程序")
        except Exception as e:
            logger.error(f"演示循环异常: {e}")
        finally:
            await self.cleanup()
    
    async def monitor_and_release(self, monitor_timeout=30.0):
        """
        监测触觉信号并在碰撞时释放
        
        Args:
            monitor_timeout (float): 监测超时时间（秒）
        """
        start_time = time.time()
        last_print = start_time
        frame_count = 0
        
        while time.time() - start_time < monitor_timeout:
            # 采集触觉数据
            tactile_data = self.tactile.collect_one_frame()
            if tactile_data is None:
                await asyncio.sleep(0.01)
                continue
            
            frame_count += 1
            self.stats['frame_count'] += 1
            
            # 检测碰撞
            is_collision = self.tactile.is_collision_detected(
                tactile_data, 
                CONFIG['collision_detection']['force_threshold']
            )
            
            # 打印统计信息
            current_time = time.time()
            if current_time - last_print >= CONFIG['monitoring']['print_interval']:
                fps = frame_count / (current_time - start_time) if (current_time - start_time) > 0 else 0
                stats = self.tactile.get_force_statistics()
                
                status_str = "⚠️  碰撞检测!" if is_collision else "✓ 正常监测"
                
                logger.info(
                    f"{status_str} | FPS: {fps:.1f} | "
                    f"力值: max={stats['max']:.2f}N, mean={stats['mean']:.2f}N, "
                    f"阈值={CONFIG['collision_detection']['force_threshold']}N"
                )
                
                last_print = current_time
            
            # 触觉可视化
            if not self.tactile.visualize():
                break
            
            # 碰撞检测 - 释放
            if is_collision:
                logger.warning("\n⚠️  检测到超过阈值的触觉力，执行释放...")
                self.stats['collision_count'] += 1
                await self.hand.release()
                self.is_grasping = False
                logger.info("✓ 释放完成")
                break
            
            # 控制帧率
            await asyncio.sleep(1.0 / CONFIG['tactile']['target_fps'])
    
    async def run_monitor_only(self):
        """
        运行纯监测模式（不执行抓握释放）
        
        用途：验证触觉传感器和阈值设置
        """
        logger.info("=" * 60)
        logger.info("启动触觉监测模式（仅监测，不控制手）")
        logger.info("=" * 60)
        
        try:
            last_print = time.time()
            frame_count = 0
            
            while True:
                tactile_data = self.tactile.collect_one_frame()
                if tactile_data is None:
                    await asyncio.sleep(0.01)
                    continue
                
                frame_count += 1
                self.stats['frame_count'] += 1
                
                is_collision = self.tactile.is_collision_detected(
                    tactile_data,
                    CONFIG['collision_detection']['force_threshold']
                )
                
                current_time = time.time()
                if current_time - last_print >= CONFIG['monitoring']['print_interval']:
                    fps = frame_count / (current_time - last_print)
                    stats = self.tactile.get_force_statistics()
                    
                    status_str = "⚠️  碰撞检测!" if is_collision else "✓ 正常"
                    
                    logger.info(
                        f"{status_str} | FPS: {fps:.1f} | "
                        f"力值: max={stats['max']:.2f}N, "
                        f"mean={stats['mean']:.2f}N"
                    )
                    
                    frame_count = 0
                    last_print = current_time
                
                if not self.tactile.visualize():
                    break
                
                await asyncio.sleep(1.0 / CONFIG['tactile']['target_fps'])
                
        except KeyboardInterrupt:
            logger.info("\n用户中断监测")
        except Exception as e:
            logger.error(f"监测异常: {e}")
        finally:
            await self.cleanup()
    
    async def cleanup(self):
        """清理资源"""
        logger.info("\n" + "=" * 60)
        logger.info("资源清理中...")
        
        # 释放手指
        if self.is_grasping:
            try:
                await self.hand.release(duration=200)
            except:
                pass
        
        # 释放触觉传感器
        self.tactile.cleanup()
        
        # 关闭Modbus连接
        try:
            libstark.modbus_close(self.hand.client)
            logger.info("Modbus连接已关闭")
        except:
            pass
        
        logger.info("=" * 60)
        logger.info("程序结束")
        logger.info(f"总帧数: {self.stats['frame_count']}")
        logger.info(f"碰撞次数: {self.stats['collision_count']}")
        logger.info("=" * 60)


# ==================== 主函数 ====================
async def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='触觉仿生手集成演示',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  # 自动检测，使用默认参数
  python revo2_tactile_grasp_demo.py
  
  # 指定手的串口和触觉传感器序列号
  python revo2_tactile_grasp_demo.py --hand-port /dev/ttyUSB0 --touch-serial BM000026
  
  # 仅监测模式（不执行抓握）
  python revo2_tactile_grasp_demo.py --mode monitor
  
  # 设置自定义阈值
  python revo2_tactile_grasp_demo.py --force-threshold 8.0
        """
    )
    
    parser.add_argument(
        '--hand-port',
        type=str,
        default=None,
        help='手的串口号（如 /dev/ttyUSB0），默认自动检测'
    )
    parser.add_argument(
        '--touch-serial',
        type=str,
        default=CONFIG['tactile']['sensor_serial'],
        help=f"触觉传感器序列号（如 BM000026），默认 {CONFIG['tactile']['sensor_serial']}"
    )
    parser.add_argument(
        '--force-threshold',
        type=float,
        default=CONFIG['collision_detection']['force_threshold'],
        help=f"触觉力阈值（牛顿），默认 {CONFIG['collision_detection']['force_threshold']}"
    )
    parser.add_argument(
        '--mode',
        type=str,
        choices=['grasp_release', 'monitor'],
        default=CONFIG['monitoring']['demo_mode'],
        help=f"运行模式，默认为 {CONFIG['monitoring']['demo_mode']} 模式"
    )
    parser.add_argument(
        '--display-tactile',
        action='store_true',
        default=CONFIG['tactile']['display_heatmap'],
        help='显示触觉热力图'
    )
    parser.add_argument(
        '--grasp-position',
        type=int,
        default=CONFIG['hand_control']['grasp_position'],
        help=f"抓握位置（0-1000），默认 {CONFIG['hand_control']['grasp_position']}"
    )
    
    args = parser.parse_args()
    
    # 根据命令行参数更新配置
    if args.force_threshold != CONFIG['collision_detection']['force_threshold']:
        CONFIG['collision_detection']['force_threshold'] = args.force_threshold
    if args.display_tactile:
        CONFIG['tactile']['display_heatmap'] = True
    if args.grasp_position != CONFIG['hand_control']['grasp_position']:
        CONFIG['hand_control']['grasp_position'] = args.grasp_position
    if args.mode != CONFIG['monitoring']['demo_mode']:
        CONFIG['monitoring']['demo_mode'] = args.mode
    
    try:
        # 1. 初始化手部控制
        logger.info("=" * 60)
        logger.info("初始化手部控制...")
        logger.info("=" * 60)
        
        client, slave_id = await open_modbus_revo2(port_name=args.hand_port)
        hand_controller = RevoHandController(client, slave_id)
        
        if not await hand_controller.initialize():
            logger.error("手部初始化失败")
            sys.exit(1)
        
        # 2. 初始化触觉采集
        logger.info("\n" + "=" * 60)
        logger.info("初始化触觉采集...")
        logger.info("=" * 60)
        
        tactile_collector = TactileDataCollector(sensor_serial=args.touch_serial)
        
        if not tactile_collector.initialize():
            logger.error("触觉采集初始化失败")
            tactile_collector.cleanup()
            libstark.modbus_close(client)
            sys.exit(1)
        
        # 3. 创建控制器并运行
        logger.info("\n" + "=" * 60)
        logger.info("初始化完成，启动主程序...")
        logger.info("=" * 60)
        
        controller = TactileGraspController(hand_controller, tactile_collector)
        
        if args.mode == 'monitor':
            await controller.run_monitor_only()
        else:
            await controller.run_grasp_release_demo()
        
    except KeyboardInterrupt:
        logger.info("\n程序被用户中断")
        sys.exit(0)
    except Exception as e:
        logger.error(f"程序异常: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
