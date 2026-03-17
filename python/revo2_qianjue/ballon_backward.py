"""
触觉仿生手 + 机械臂：气球主动推针（单次扰动步进后退版）Demo
"""

import asyncio
import time
import sys
import argparse
from pathlib import Path
import numpy as np
import yaml
# 动态导入路径
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import logger
# ==================== 动态加载新配置 ====================
config_file = Path(__file__).parent / "config_backward.yaml"
if not config_file.exists():
    logger.error("❌ 找不到 config_backward.yaml，请检查文件是否存在！")
    sys.exit(1)
with open(config_file, 'r', encoding='utf-8') as f:
    NEW_CONFIG = yaml.safe_load(f)

import revo2_tactile_grasp_demo
revo2_tactile_grasp_demo.CONFIG.update(NEW_CONFIG)
from revo2_tactile_grasp_demo import TactileDataCollector, RevoHandController, open_modbus_revo2
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
from xensesdk import Sensor
import cv2

# ==================== 触觉采集模块 (Difference 特化版) ====================
class DiffTactileDataCollector(TactileDataCollector):
    """继承原有的采集器，强制改为采集 Difference (差异) 数据以提高鲁棒性"""

    def collect_one_frame(self):
        if not self.sensor: return None
        try:
            # 1. 获取 BGR 格式的差分图，shape=(700, 400, 3)
            raw_diff = self.sensor.selectSensorInfo(Sensor.OutputType.Difference)
            if raw_diff is None: return None

            # 2. 直接转为单通道灰度图来计算全图的受力平均值，绝不要再乘以 200！
            diff_gray = cv2.cvtColor(raw_diff, cv2.COLOR_BGR2GRAY)
            current_score = float(np.mean(diff_gray))

            # 3. 画面显示依然保留彩色原始图，方便你观察
            self.display_img = raw_diff

            return {
                'force_val': current_score,
                'timestamp': time.time()
            }
        except Exception as e:
            logger.error(f"采集触觉 Difference 数据异常: {e}")
            return None


# ==================== 机械臂控制模块 ====================
class YieldingArmController:
    def __init__(self, arm_ip, arm_port):
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        logger.info(f"正在尝试连接机械臂: {arm_ip}:{arm_port} ...")
        self.handle = self.arm.rm_create_robot_arm(str(arm_ip).strip(), int(arm_port))
        self.connected = self.handle.id != -1

        if self.connected:
            logger.info(f"✅ 成功连接机械臂，ID: {self.handle.id}")
            self.home_joint = NEW_CONFIG['arm_control']['home_joint_angles']
        else:
            logger.error(f"❌ 无法连接到机械臂 {arm_ip}:{arm_port}")

    async def return_to_home(self):
        if not self.connected: return
        speed = NEW_CONFIG['arm_control']['return_speed']
        logger.info(f"🔄 机械臂正前往靠前的初始发车点 (速度: {speed}%)...")
        ret = await asyncio.to_thread(self.arm.rm_movej, self.home_joint, speed, 0, 0, 1)
        if ret == 0:
            logger.info("✅ 已到达初始位置！")

    def stop_move(self):
        if self.connected:
            self.arm.rm_set_arm_stop()
            self.arm.rm_set_arm_delete_trajectory()

    def get_current_x(self):
        """安全获取当前X坐标"""
        if not self.connected: return None
        ret, state = self.arm.rm_get_current_arm_state()
        if ret == 0:
            return state['pose'][0]
        return None

    async def start_yielding_to_target(self, target_x, speed):
        """向目标坐标滑动 (异步非阻塞)"""
        if not self.connected: return
        ret_state, state = self.arm.rm_get_current_arm_state()
        if ret_state != 0: return

        curr_pose = state['pose']
        # 发送后退指令 (+X 方向)，block=1 确保退完这 5cm 再继续
        new_pose = [
            target_x,
            curr_pose[1], curr_pose[2], curr_pose[3], curr_pose[4], curr_pose[5]
        ]
        await asyncio.to_thread(self.arm.rm_movel, new_pose, speed, 0, 0, 0)

    def cleanup(self):
        if self.connected:
            self.stop_move()
            self.arm.rm_delete_robot_arm()


# ==================== 核心逻辑控制器 ====================
class ContinuousYieldingController:
    def __init__(self, hand_controller, tactile_collector, arm_controller):
        self.hand = hand_controller
        self.tactile = tactile_collector
        self.arm = arm_controller

    async def run_demo(self):
        logger.info(">>> 🎈 启动【气球扰动：连续柔顺退让 Demo (Difference版)】")

        logger.info(">>> [初始化] 正在用 OK 手势抓牢针头...")
        await asyncio.sleep(1.0)
        await self.hand.grasp()
        await asyncio.sleep(1.0)

        try:
            # while True:
            await self.arm.return_to_home()
            await asyncio.sleep(1.0)

            # ================= Difference 校准逻辑 =================
            logger.info(">>> [校准中] 等待触觉传感器 Difference 唤醒...")
            calibration_scores = []

            for _ in range(30):
                self.tactile.collect_one_frame()
                await asyncio.sleep(0.02)

            give_up_time = time.time() + 5.0
            while len(calibration_scores) < 50:
                data = self.tactile.collect_one_frame()
                if data is not None:
                    val = data['force_val']
                    calibration_scores.append(val)
                    self.tactile.visualize()

                if time.time() > give_up_time:
                    break
                await asyncio.sleep(0.01)

            self.tactile.baseline_score = np.mean(calibration_scores) if calibration_scores else 0.0
            self.tactile.score_history.clear()

            th = NEW_CONFIG['collision_detection']['force_threshold']
            logger.info(f">>> ✅ Difference 基准已锁定: {self.tactile.baseline_score:.1f} | 敏感阈值: ±{th:.1f}")
            logger.info(">>> 💡 待命中！拿着气球抵住针头，只要保持推力，机械臂就会一直连续后退！")

            # 运行连续柔顺监测逻辑
            await self.monitor_continuous_yielding()

            logger.info(">>> 🎬 全局限位已到达，本轮拍摄完毕！3 秒后自动归位...\n")
            await asyncio.sleep(3.0)

        # except KeyboardInterrupt:
            logger.info("\n检测到退出指令，正在终止程序...")
        finally:
            await self.cleanup()

    async def monitor_continuous_yielding(self):
        """状态机：推力超过阈值 -> 连续退让 -> 推力消失/跟不上 -> 立刻刹车"""
        th_trigger = NEW_CONFIG['collision_detection']['force_threshold']
        # 释放阈值设为触发阈值的一半。偏差低于此值则视为脱离
        th_release = th_trigger * 0.5

        ret_speed = NEW_CONFIG['arm_control']['retreat_speed']
        max_dist = NEW_CONFIG['arm_control'].get('max_retreat_distance', 0.30)
        ret_speed = NEW_CONFIG['arm_control']['retreat_speed']

        # 记录初始原点坐标，计算绝对终点
        start_x = self.arm.get_current_x()
        if start_x is None: start_x = 0.0
        limit_x = start_x + max_dist

        # 🌟 修复点：明确在此处初始化状态变量，绝对不能缩进到 while True 里面！
        is_yielding = False
        last_print = time.time()
        cooldown_until = 0.0  # 🌟 物理冷却时间戳

        while True:
            data = self.tactile.collect_one_frame()
            if data is None:
                await asyncio.sleep(0.01)
                continue

            current_force = data['force_val']
            deviation = abs(current_force - self.tactile.baseline_score)

            if not self.tactile.visualize():
                sys.exit(0)

            # 获取当前机械臂位置
            curr_x = self.arm.get_current_x()

            # ================= 2. 正常监测期 =================
            if time.time() - last_print > 0.5:
                status = "🔴 受推力，持续退让中..." if is_yielding else "🟢 待命中，等待推入..."
                logger.info(f"{status} | Diff差值: {current_force:.1f} | 零位: {self.tactile.baseline_score:.1f} | 偏差: {deviation:.1f}")
                last_print = time.time()

            # 全局极限保护：不管什么状态，退到底线直接结束本轮
            if curr_x is not None and curr_x >= limit_x - 0.005:
                logger.info("🛑 已经退到了全局最大极限距离，停止后退！")
                self.arm.stop_move()
                return

            # ================= 连续柔顺状态机 =================
            if not is_yielding:
                if deviation > th_trigger:
                    logger.warning(f"⚠️ 受到持续挤压！偏差 {deviation:.1f}，开启连续退让模式...")
                    self.arm.stop_move()
                    await asyncio.sleep(0.02)

                    # 设定终极目标 limit_x，只要不喊停，它就会一直平滑滑向终点
                    await self.arm.start_yielding_to_target(target_x=limit_x, speed=ret_speed)
                    is_yielding = True
                else:
                    # 待命时的缓慢自动归零
                    self.tactile.baseline_score = self.tactile.baseline_score * 0.95 + current_force * 0.05
            else:
                # 正在后退中：如果气球拿开了，或者推的速度比机械臂退的速度慢，导致压力下降
                if deviation < th_release:
                    logger.info("✅ 压力减弱或消失，立即刹车！")
                    self.arm.stop_move()
                    is_yielding = False

                    # 刹车后立刻将当前数值设为新基准，防余震
                    self.tactile.baseline_score = current_force

            await asyncio.sleep(0.01)

    async def cleanup(self):
        self.arm.stop_move()
        self.tactile.cleanup()
        self.arm.cleanup()
        try:
            from common_imports import libstark
            libstark.modbus_close(self.hand.client)
        except:
            pass


# ==================== 程序入口 ====================
async def main():
    default_ip = NEW_CONFIG['arm_control']['arm_ip']
    default_port = NEW_CONFIG['arm_control']['arm_port']

    parser = argparse.ArgumentParser(description='气球推针单步退让 Demo')
    parser.add_argument('--hand-port', type=str, help='手部串口')
    parser.add_argument('--touch-serial', type=str, default=NEW_CONFIG['tactile']['sensor_serial'], help='触觉序列号')
    parser.add_argument('--arm-ip', type=str, default=default_ip)
    parser.add_argument('--arm-port', type=int, default=default_port)

    args = parser.parse_args()

    client, slave_id = await open_modbus_revo2(port_name=args.hand_port)
    hand = RevoHandController(client, slave_id)
    await hand.initialize()

    # 🌟 使用重写的 Difference 采集器
    tactile = DiffTactileDataCollector(args.touch_serial)
    if not tactile.initialize(): sys.exit(1)

    arm = YieldingArmController(args.arm_ip, args.arm_port)
    controller = ContinuousYieldingController(hand, tactile, arm)

    try:
        await controller.run_demo()
    except Exception as e:
        logger.error(f"运行出错: {e}")
        await controller.cleanup()

if __name__ == "__main__":
    asyncio.run(main())