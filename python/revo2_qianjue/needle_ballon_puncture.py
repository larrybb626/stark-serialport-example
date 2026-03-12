"""
触觉仿生手 + 机械臂集成模块 - 垂直扎气球避险避让 Demo (Z轴专属版)
"""



import asyncio
import time
import sys
import argparse
from pathlib import Path
import numpy as np
import yaml

# 导入原有环境与配置
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import logger
from revo2_utils import open_modbus_revo2

# ==================== 日志简洁化配置 ====================
import logging
import os
class ConciseFormatter(logging.Formatter):
    """自定义格式化器：将绝对路径简化为 ~/ 或 仅文件名"""
    def format(self, record):
        record.concise_path = os.path.basename(record.pathname)
        return super().format(record)

for handler in logger.handlers:
    # 定义简洁的格式：时间 [等级] [路径:行号] 消息
    new_fmt = '%(asctime)s [%(levelname)s] [%(concise_path)s:%(lineno)d] %(message)s'
    handler.setFormatter(ConciseFormatter(new_fmt))
# ===================================================

# 导入已实现好的手部和触觉控制模块，包含 CONFIG
from revo2_tactile_grasp_demo import (
    TactileDataCollector,
    RevoHandController,
    TactileGraspController,
    CONFIG
)

# 导入机械臂接口
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


# ==================== 机械臂控制模块 ====================
class ArmController:
    """包装机械臂API，将其桥接到 asyncio 环境下"""

    def __init__(self, arm_ip, arm_port):
        clean_ip = str(arm_ip).strip()
        clean_port = int(arm_port)

        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

        logger.info(f"正在尝试连接机械臂: {clean_ip}:{clean_port} ...")
        self.handle = self.arm.rm_create_robot_arm(clean_ip, clean_port)
        self.connected = False

        if self.handle.id == -1:
            logger.error(f"无法连接到机械臂 {clean_ip}:{clean_port}")
        else:
            self.connected = True
            logger.info(f"✅ 成功连接机械臂: {clean_ip}，ID: {self.handle.id}")
            # 🌟 新逻辑：加载配置文件中的绝对原点
            self.setup_home_position()

    def setup_home_position(self):
        """设定安全的初始原点位置"""
        arm_cfg = CONFIG.get('arm_control', {})
        home_angles = arm_cfg.get('home_joint_angles', None)

        if home_angles and len(home_angles) == 6:
            self.home_joint = home_angles
            logger.info(f"🏠 已加载配置文件中的固定安全原点 (Joints): {[round(j, 3) for j in self.home_joint]}")
        else:
            # 兼容处理：如果没有配置，则像以前一样记录当前位置
            ret, state = self.arm.rm_get_current_arm_state()
            if ret == 0:
                self.home_joint = state['joint']
                logger.info(f"🏠 配置文件未指定角度，已锁定当前随意位置为原点 (Joints): {[round(j, 3) for j in self.home_joint]}")
            else:
                logger.error(f"❌ 记录初始位置失败，错误码: {ret}")

    async def return_to_home(self):
        """让机械臂安全返回记录的初始位置"""
        if not self.connected or not hasattr(self, 'home_joint'): return

        # 动态读取 YAML 中的回零速度配置，默认 20
        speed = CONFIG.get('arm_control', {}).get('return_speed', 20)
        logger.info(f"🔄 机械臂正在平稳返回固定初始原点 (速度: {speed}%)...")
        # rm_movej 用于回到指定的各个关节角度，这能保证机械臂不仅位置对，而且连手肘的姿态也和原来一模一样
        ret = await asyncio.to_thread(self.arm.rm_movej, self.home_joint, speed, 0, 0, 1)
        if ret == 0:
            logger.info("✅ 已安全回到绝对初始原点！")
        else:
            logger.error(f"❌ 返回原点失败，错误码: {ret}")

    async def move_forward_async(self, distance, speed):
        """水平向前移动 (异步非阻塞)"""
        if not self.connected: return
        logger.info(f"➡️ 机械臂开始水平向前推进 {distance * 100}cm (速度: {speed}%)...")

        ret_state, state = self.arm.rm_get_current_arm_state()
        if ret_state != 0: return

        curr_pose = state['pose']

        # ✅ 前进方向修正：减去距离 (-X) 代表向正前方冲锋
        new_pose = [
            curr_pose[0] - distance,
            curr_pose[1], curr_pose[2], curr_pose[3], curr_pose[4], curr_pose[5]
        ]

        ret = await asyncio.to_thread(self.arm.rm_movel, new_pose, speed, 0, 0, 0)
        if ret != 0: logger.error(f"❌ 前进指令发送失败，错误码: {ret}")

    def stop_move(self):
        """🚨 【新增】一键急停功能，打断当前所有运动队列"""
        if self.connected:
            self.arm.rm_set_arm_stop()               # 瞬间刹停
            self.arm.rm_set_arm_delete_trajectory()  # 清空后续缓存的运动轨迹

    async def emergency_retreat_backward(self, distance, speed):
        """紧急水平向后回退 (阻塞等待)"""
        if not self.connected: return

        # 在执行后退前，再确保一次机械臂已经完全静止，避免读取坐标出现漂移
        self.stop_move()
        await asyncio.sleep(0.05)

        logger.info(f"⬅️ 触发避险！向后撤离 {distance * 100}cm (速度: {speed}%)...")

        ret_state, state = self.arm.rm_get_current_arm_state()
        if ret_state != 0: return

        curr_pose = state['pose']

        # ✅ 后退方向修正：加上距离 (+X) 代表向后撤离
        new_pose = [
            curr_pose[0] + distance,
            curr_pose[1], curr_pose[2], curr_pose[3], curr_pose[4], curr_pose[5]
        ]

        ret = await asyncio.to_thread(self.arm.rm_movel, new_pose, speed, 0, 0, 1)

    def cleanup(self):
        """清理资源，断开机械臂"""
        if self.connected:
            self.arm.rm_delete_robot_arm()
            logger.info("已断开机械臂连接")


# ==================== 继承与重写主逻辑 ====================
class IntegratedGraspController(TactileGraspController):
    """
    继承原有的控制器。
    直接复用父类的 monitor_collision_loop 等逻辑，仅重写主循环来加入机械臂的联动。
    """

    def __init__(self, hand_controller, tactile_collector, arm_controller):
        # 初始化父类（手部与触觉）
        super().__init__(hand_controller, tactile_collector)
        # 挂载机械臂
        self.arm = arm_controller

    async def run_grasp_release_demo(self):
        logger.info(">>> 启动【水平扎破方案】：[抓握 -> 向前推进 -> 运动中监测 -> 触发反弹 -> 归零]")

        # 预先从 CONFIG 读取所有参数 (带有防呆默认值)
        arm_cfg = CONFIG.get('arm_control', {})
        fwd_dist = arm_cfg.get('forward_distance', 0.10)
        fwd_spd = arm_cfg.get('forward_speed', 15)
        ret_dist = arm_cfg.get('retreat_distance', 0.10)
        ret_spd = arm_cfg.get('retreat_speed', 50)

        # 1. 第一步：高空抓握针头
        logger.info(">>> [初始化] 正在用 OK 手势抓牢针头...")
        await self.hand.grasp()
        await asyncio.sleep(1)

        try:
            # while True:
            # 0. 确保手是松开的，并退回到安全高空
            # await self.hand.release()
            await self.arm.return_to_home()
            await asyncio.sleep(1.0)

            # 2. 第二步：发送前进指令 (调用 YAML 配置的距离和速度)
            logger.info(">>> [2/5] 机械臂开始向前推进...")
            await self.arm.move_forward_async(distance=fwd_dist, speed=fwd_spd)

            # 3. 第三步：在行进中过滤启动震动，提取基准
            logger.info(">>> [3/5] 进入运动校准期(3.5秒)：忽略启动震动，提取平稳前进基准...")
            calibration_scores = []
            calib_start = time.time()

            while time.time() - calib_start < 2:
                data = self.tactile.collect_one_frame()
                if data is not None:
                    # 前 1 秒机械臂刚启动时的震动数据被丢弃，只取后 1 秒的平滑数据
                    if time.time() - calib_start > 1:
                        calibration_scores.append(data['force_val'])
                    self.tactile.visualize()
                await asyncio.sleep(0.01)

            if len(calibration_scores) > 10:
                self.tactile.baseline_score = np.mean(calibration_scores)
            else:
                self.tactile.baseline_score = 0.0

            self.tactile.score_history.clear()
            th = CONFIG.get('collision_detection', {}).get('force_threshold', 1.0)

            logger.info(f">>> 动态基准锁定为: {self.tactile.baseline_score:.1f} | 触发阈值: ±{th:.1f}")
            logger.info(">>> 监测正式开始，等待气球爆破扰动信号...")

            # 4. 带超时的运动监测
            triggered = await self.monitor_collision_loop_with_timeout(timeout=10.0)

            # 5. 触发联动避让
            if triggered:
                logger.warning(">>> [4/5] ⚠️ 检测触碰到气球！立即打断移动，执行避险动作...")

                # 🛑
                self.arm.stop_move()

                # await self.hand.release()

                # 调用 YAML 配置的后退距离和速度
                await self.arm.emergency_retreat_backward(distance=ret_dist, speed=ret_spd)
                self.tactile.baseline_score = 0.0

                logger.info(">>> [5/5] 避险后退完成，保持悬停 2 秒供观察...")
                await asyncio.sleep(2.0)
            else:
                logger.info(">>> [4/5] 机械臂已到底部，未检测到气球爆炸...")

            # logger.info("--- 本轮循环结束，准备回归原点并重试 ---\n")
            await asyncio.sleep(1.0)

        except KeyboardInterrupt:
            logger.info("\n检测到退出指令，正在终止程序...")
        finally:
            await self.cleanup()

    async def monitor_collision_loop_with_timeout(self, timeout=10.0):
        """新增带超时机制的监测循环，防止没碰到气球导致一直卡住"""
        start_time = time.time()
        last_print = time.time()
        # th = CONFIG['collision_detection']['force_threshold']

        while time.time() - start_time < timeout:
            data = self.tactile.collect_one_frame()
            if data is None:
                await asyncio.sleep(0.01)
                continue

            is_collision = self.tactile.is_collision_detected(data)

            if time.time() - last_print > 1.0:
                logger.info(f"持续前进监测中... | 当前受力: {data['force_val']:.1f}")
                last_print = time.time()

            if not self.tactile.visualize():
                return False

            if is_collision:
                return True

            await asyncio.sleep(0.01)

        return False

    async def cleanup(self):
        await super().cleanup()
        self.arm.cleanup()


# ==================== 程序入口 ====================
async def main():
    # 从配置加载默认 IP 和端口
    arm_cfg = CONFIG.get('arm_control', {})
    default_ip = arm_cfg.get('arm_ip', '192.168.1.18')
    default_port = arm_cfg.get('arm_port', 8080)

    parser = argparse.ArgumentParser(description='Revo2 触觉抓握及机械臂联动演示')
    parser.add_argument('--hand-port', type=str, help='手部串口')
    parser.add_argument('--touch-serial', type=str, default=CONFIG['tactile']['sensor_serial'], help='触觉序列号')
    parser.add_argument('--mode', type=str, choices=['grasp', 'monitor'], default='grasp')

    # 支持命令行参数覆盖 YAML 的 IP 设定
    parser.add_argument('--arm-ip', type=str, default=default_ip, help='机械臂IP地址')
    parser.add_argument('--arm-port', type=int, default=default_port, help='机械臂控制端口')

    args = parser.parse_args()

    # 1. 初始化手部
    client, slave_id = await open_modbus_revo2(port_name=args.hand_port)
    hand = RevoHandController(client, slave_id)
    await hand.initialize()

    # 2. 初始化触觉
    tactile = TactileDataCollector(args.touch_serial)
    if not tactile.initialize(): sys.exit(1)

    # 3. 初始化机械臂
    arm = ArmController(args.arm_ip, args.arm_port)

    # 4. 载入综合控制器 (子类化)
    controller = IntegratedGraspController(hand, tactile, arm)

    try:
        if args.mode == 'monitor':
            # 直接复用父类的监测模式
            await controller.run_monitor_only()
        else:
            # 运行我们重写后的带机械臂后退的闭环演示
            await controller.run_grasp_release_demo()
    except Exception as e:
        logger.error(f"运行出错: {e}")
        await controller.cleanup()


if __name__ == "__main__":
    asyncio.run(main())