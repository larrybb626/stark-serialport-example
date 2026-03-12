"""
触觉仿生手 + 机械臂：气球主动推针（柔顺退让）Demo
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

# 巧妙复用：导入之前的模块，并强制将其内部的 CONFIG 更新为我们的新配置
import revo2_tactile_grasp_demo

revo2_tactile_grasp_demo.CONFIG.update(NEW_CONFIG)
from revo2_tactile_grasp_demo import TactileDataCollector, RevoHandController, open_modbus_revo2
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


# ==================== 机械臂控制模块 (柔顺特化版) ====================
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
        """前往靠前的初始接气球位置"""
        if not self.connected: return
        speed = NEW_CONFIG['arm_control']['return_speed']
        logger.info(f"🔄 机械臂正前往靠前的初始原点 (速度: {speed}%)...")
        ret = await asyncio.to_thread(self.arm.rm_movej, self.home_joint, speed, 0, 0, 1)
        if ret == 0:
            logger.info("✅ 已到达初始位置，等待气球推入！")

    def stop_move(self):
        """一键急停，打断当前的滑动"""
        if self.connected:
            self.arm.rm_set_arm_stop()
            self.arm.rm_set_arm_delete_trajectory()

    async def start_yielding_backward(self, max_distance, speed):
        """开启非阻塞的向后滑动"""
        if not self.connected: return
        ret_state, state = self.arm.rm_get_current_arm_state()
        if ret_state != 0: return

        curr_pose = state['pose']
        # 发送一个长距离的后退指令 (+X 方向)，block=0 让它在后台自己滑，不卡死 Python
        new_pose = [
            curr_pose[0] + max_distance,
            curr_pose[1], curr_pose[2], curr_pose[3], curr_pose[4], curr_pose[5]
        ]
        await asyncio.to_thread(self.arm.rm_movel, new_pose, speed, 0, 0, 0)

    def cleanup(self):
        if self.connected:
            self.stop_move()
            self.arm.rm_delete_robot_arm()


# ==================== 核心逻辑控制器 ====================
class YieldingDemoController:
    def __init__(self, hand_controller, tactile_collector, arm_controller):
        self.hand = hand_controller
        self.tactile = tactile_collector
        self.arm = arm_controller

    async def run_demo(self):
        logger.info(">>> 🎈 启动【气球推针柔顺退让 Demo】")

        # 1. 抓死针头，不再松开
        logger.info(">>> [初始化] 正在用 OK 手势抓牢针头...")
        # await self.hand.release()
        await asyncio.sleep(1.0)
        await self.hand.grasp()
        await asyncio.sleep(1.0)

        try:
            # 2. 前往发车点
            await self.arm.return_to_home()
            await asyncio.sleep(1.0)

            # ================= 修复：带超时和打印的校准逻辑 =================
            logger.info(">>> [校准中] 等待触觉传感器唤醒...")
            calibration_scores = []

            # 1. 先空跑 30 帧 (约 1 秒)
            for _ in range(30):
                self.tactile.collect_one_frame()
                await asyncio.sleep(0.02)

            # 2. 尝试收集有效数据，但如果 5 秒内还是收集不到，就强制通过
            give_up_time = time.time() + 5.0
            while len(calibration_scores) < 50:
                data = self.tactile.collect_one_frame()
                if data is not None:
                    val = data['force_val']
                    # 打印当前读数，方便你观察它是不是真的是 0
                    if len(calibration_scores) % 10 == 0:
                        logger.info(f"正在收集校准数据... 当前实时值: {val:.2f}")

                    # 只要有数据就录入，不再强求 > 0.1 (如果真的是0，那就以0为基准)
                    calibration_scores.append(val)
                    self.tactile.visualize()

                if time.time() > give_up_time:
                    logger.warning("⚠️ 传感器唤醒超时，将使用已有数据强制计算基准")
                    break
                await asyncio.sleep(0.01)

            # 如果完全没收集到数据，给个默认 0
            self.tactile.baseline_score = np.mean(calibration_scores) if calibration_scores else 0.0
            # ===========================================================
            self.tactile.score_history.clear()

            th = NEW_CONFIG['collision_detection']['force_threshold']
            logger.info(f">>> ✅ 校准完成！真实基准已锁定为: {self.tactile.baseline_score:.1f} | 敏感阈值: ±{th:.1f}")
            logger.info(">>> 💡 请现在拿着气球，慢慢抵住针头并往前推...")

            # 4. 进入状态机死循环，实时退让
            await self.monitor_yielding_loop()

        except KeyboardInterrupt:
            logger.info("\n检测到退出指令，正在终止程序...")
        finally:
            await self.cleanup()

    async def monitor_yielding_loop(self):
        """核心状态机：引入动态基准与峰值回落检测"""
        th_trigger = NEW_CONFIG['collision_detection']['force_threshold']
        # 释放阈值：只要压力比峰值下降了 50% 的触发阈值，就认为气球脱离了
        th_release = th_trigger * 0.5

        ret_speed = NEW_CONFIG['arm_control']['retreat_speed']
        max_dist = NEW_CONFIG['arm_control']['max_retreat_distance']

        is_yielding = False
        peak_deviation = 0.0  # 记录推压过程中的最大形变偏差
        last_print = time.time()

        # 🌟 新增：强制滑行时间锁（秒）。触发后退后，至少滑行这么久才允许刹车。
        min_yield_time = 0.4
        yield_start_time = 0.0

        while True:
            data = self.tactile.collect_one_frame()
            if data is None:
                await asyncio.sleep(0.01)
                continue

            current_force = data['force_val']
            # 计算当前力与动态基准的绝对偏差
            deviation = abs(current_force - self.tactile.baseline_score)

            # 控制台状态打印 (半秒刷新一次)
            if time.time() - last_print > 0.5:
                status = "🔴 受推力，滑动退让中..." if is_yielding else "🟢 待命中，等待推入..."
                logger.info(f"{status} | 当前力: {current_force:.1f} | 零位: {self.tactile.baseline_score:.1f} | 偏差: {deviation:.1f}")
                last_print = time.time()

            if not self.tactile.visualize():
                break

            # ===== 🌟 柔顺状态机核心逻辑 =====
            if not is_yielding:
                if deviation > th_trigger:
                    # 刚碰到气球：偏差超过阈值，触发后退
                    self.arm.stop_move()
                    await asyncio.sleep(0.02)
                    await self.arm.start_yielding_backward(max_distance=max_dist, speed=ret_speed)
                    is_yielding = True
                    peak_deviation = deviation
                    yield_start_time = time.time()  # 记录开始滑动的时间戳
                else:
                    # 【神级优化】：待命状态下，让基准值极缓慢地跟随当前受力（指数移动平均滤波）
                    # 这样可以自动消除因为之前推压导致的针头微小错位残余力！
                    self.tactile.baseline_score = self.tactile.baseline_score * 0.95 + current_force * 0.05
            else:
                # 正在后退滑动中
                peak_deviation = max(peak_deviation, deviation) # 持续刷新最大推力

                # 脱离判定：如果偏差从峰值回落了（气球停止推进或拿开）
                if deviation < peak_deviation - th_release:
                    if time.time() - yield_start_time > min_yield_time:
                        # 只有滑够了时间，才允许真正刹车
                        self.arm.stop_move()
                        is_yielding = False

                        self.tactile.baseline_score = current_force
                        peak_deviation = 0.0

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

    parser = argparse.ArgumentParser(description='气球主动推针（柔顺退让）Demo')
    parser.add_argument('--hand-port', type=str, help='手部串口')
    parser.add_argument('--touch-serial', type=str, default=NEW_CONFIG['tactile']['sensor_serial'], help='触觉序列号')
    parser.add_argument('--arm-ip', type=str, default=default_ip)
    parser.add_argument('--arm-port', type=int, default=default_port)

    args = parser.parse_args()

    client, slave_id = await open_modbus_revo2(port_name=args.hand_port)
    hand = RevoHandController(client, slave_id)
    await hand.initialize()

    tactile = TactileDataCollector(args.touch_serial)
    if not tactile.initialize(): sys.exit(1)

    arm = YieldingArmController(args.arm_ip, args.arm_port)
    controller = YieldingDemoController(hand, tactile, arm)

    try:
        await controller.run_demo()
    except Exception as e:
        logger.error(f"运行出错: {e}")
        await controller.cleanup()


if __name__ == "__main__":
    asyncio.run(main())