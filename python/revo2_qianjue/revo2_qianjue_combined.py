"""
触觉仿生手 + 机械臂：【扎破气球 -> 战术反弹 -> 原点复位 -> 柔顺退让】 全新综合连招 Demo
"""

import asyncio
import time
import sys
import argparse
from pathlib import Path
import numpy as np
import yaml
import cv2

# 动态导入路径
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import logger

# ==================== 1. 干净地加载两套配置 (彻底解决 Config 错乱) ====================
combined_cfg_path = Path(__file__).parent / "config_combined.yaml"
if not combined_cfg_path.exists():
    logger.error("❌ 找不到 config_combined.yaml！")
    sys.exit(1)

with open(combined_cfg_path, 'r', encoding='utf-8') as f:
    loader = yaml.safe_load(f)

# 分别把两套配置加载为两个独立的字典，互不干扰
with open(Path(__file__).parent / loader['needle_config'], 'r', encoding='utf-8') as f:
    CFG_PHASE1 = yaml.safe_load(f)
with open(Path(__file__).parent / loader['backward_config'], 'r', encoding='utf-8') as f:
    CFG_PHASE2 = yaml.safe_load(f)

# 导入底层硬件 SDK（不导入带有副作用的业务脚本）
import revo2_tactile_grasp_demo
revo2_tactile_grasp_demo.CONFIG.update(CFG_PHASE1) # 给底层包一个默认值，防止报错
from revo2_tactile_grasp_demo import TactileDataCollector, RevoHandController, open_modbus_revo2
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

try:
    from xensesdk import Sensor
except ImportError:
    pass

# ==================== 触觉采集模块 (支持双模式热切换) ====================
class SwitchableTactileCollector(TactileDataCollector):
    """全局统一持有相机资源，但支持根据阶段切换 Depth 和 Difference 模式"""
    def __init__(self, sensor_serial=None):
        super().__init__(sensor_serial)
        self.use_diff = False  # 默认为 False，即使用原版 Depth 模式

    def collect_one_frame(self):
        if not self.sensor: return None
        try:
            # 🌟 核心修改：无论处于哪个阶段，我们都提取 Depth 图像用于显示
            # 这样就能保证 OpenCV 窗口从头到尾都是同一种蓝色的视觉风格
            raw_depth = self.sensor.selectSensorInfo(Sensor.OutputType.Depth)
            depth_vis = None
            if raw_depth is not None:
                depth_vis = np.clip(raw_depth * 200, 0, 255).astype(np.uint8)

            if self.use_diff:
                # ====== Phase 2: Difference 模式 ======
                raw_diff = self.sensor.selectSensorInfo(Sensor.OutputType.Difference)
                if raw_diff is None: return None
                diff_gray = cv2.cvtColor(raw_diff, cv2.COLOR_BGR2GRAY)
                current_score = float(np.mean(diff_gray))

                # 显示画面使用 Depth (障眼法)
                self.display_img = depth_vis if depth_vis is not None else raw_diff
            else:
                # ====== Phase 1: 完美复原原版 Depth 模式 ======
                raw_depth = self.sensor.selectSensorInfo(Sensor.OutputType.Depth)
                if raw_depth is None: return None
                depth_vis = np.clip(raw_depth * 200, 0, 255).astype(np.uint8)
                current_score = float(np.mean(depth_vis))
                self.display_img = depth_vis

            return {'force_val': current_score, 'timestamp': time.time()}
        except Exception as e:
            logger.error(f"采集触觉数据异常: {e}")
            return None

class UnifiedArmController:
    """一个包含所有动作的综合机械臂控制器"""
    def __init__(self, arm_ip, arm_port):
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        logger.info(f"正在尝试连接机械臂: {arm_ip}:{arm_port} ...")
        self.handle = self.arm.rm_create_robot_arm(str(arm_ip).strip(), int(arm_port))
        self.connected = self.handle.id != -1
        self.current_home_joint = None

        if self.connected:
            logger.info(f"✅ 成功连接机械臂，ID: {self.handle.id}")
        else:
            logger.error(f"❌ 无法连接到机械臂 {arm_ip}:{arm_port}")

    def set_home_joint(self, joint_angles):
        """动态切换 Home 坐标"""
        self.current_home_joint = joint_angles
        logger.info(f"🏠 当前 Home 坐标已切换为: {[round(j, 3) for j in joint_angles]}")

    async def return_to_home(self, speed):
        if not self.connected or not self.current_home_joint: return
        logger.info(f"🔄 机械臂正前往 Home 发车点 (速度: {speed}%)...")
        ret = await asyncio.to_thread(self.arm.rm_movej, self.current_home_joint, speed, 0, 0, 1)
        if ret == 0: logger.info("✅ 已到达发车位置！")

    def stop_move(self):
        if self.connected:
            self.arm.rm_set_arm_stop()
            self.arm.rm_set_arm_delete_trajectory()

    def get_current_x(self):
        if not self.connected: return None
        ret, state = self.arm.rm_get_current_arm_state()
        if ret == 0: return state['pose'][0]
        return None

    # --- Phase 1 动作 ---
    async def move_forward_async(self, distance, speed):
        if not self.connected: return
        ret, state = self.arm.rm_get_current_arm_state()
        if ret != 0: return
        new_pose = state['pose']
        new_pose[0] -= distance # 向前
        await asyncio.to_thread(self.arm.rm_movel, new_pose, speed, 0, 0, 0)

    async def emergency_retreat_backward(self, distance, speed):
        if not self.connected: return
        self.stop_move()
        await asyncio.sleep(0.05)
        ret, state = self.arm.rm_get_current_arm_state()
        if ret != 0: return
        new_pose = state['pose']
        new_pose[0] += distance # 向后反弹
        await asyncio.to_thread(self.arm.rm_movel, new_pose, speed, 0, 0, 1)

    # --- Phase 2 动作 ---
    async def start_yielding_to_target(self, target_x, speed):
        if not self.connected: return
        ret, state = self.arm.rm_get_current_arm_state()
        if ret != 0: return
        new_pose = state['pose']
        new_pose[0] = target_x # 平滑向目标滑动
        await asyncio.to_thread(self.arm.rm_movel, new_pose, speed, 0, 0, 0)

    def cleanup(self):
        if self.connected:
            self.stop_move()
            self.arm.rm_delete_robot_arm()

# ==================== 3. 终极状态机 (解决复位和衔接逻辑) ====================
class MasterDemoController:
    def __init__(self, hand, tactile, arm):
        self.hand = hand
        self.tactile = tactile
        self.arm = arm

    async def run_full_sequence(self):
        logger.info("\n" + "★"*40 + "\n>>> 🎬 启动终极连招：[带针冲锋 -> 战术反弹 -> 原点复位 -> 柔顺退让]\n" + "★"*40)

        # 抓紧针头，死不松手
        logger.info(">>> [准备] 正在用 OK 手势抓牢针头...")
        await self.hand.grasp()
        await asyncio.sleep(1.0)

        try:
            while True:
                # ==================== Phase 1：主动出击 ====================
                logger.info("\n" + "="*30 + " 【Phase 1：主动冲锋扎破】 " + "="*30)
                # 切换到 Needle 的坐标和参数
                p1_cfg = CFG_PHASE1['arm_control']
                self.arm.set_home_joint(p1_cfg['home_joint_angles'])
                await self.arm.return_to_home(p1_cfg['return_speed'])
                await asyncio.sleep(1.0)

                hit_success = await self.execute_phase1(p1_cfg, CFG_PHASE1['collision_detection']['force_threshold'])

                # ==================== 完美复位衔接 ====================
                if hit_success:
                    logger.info(">>> [衔接] 扎破气球并已战术后退。保持镜头感 2 秒...")
                    await asyncio.sleep(5.0)

                    logger.info(">>> [衔接] 正在将机械臂安全复位到 Phase 2 发车点...")
                    # 切换到 Backward 的坐标
                    p2_cfg = CFG_PHASE2['arm_control']
                    self.arm.set_home_joint(p2_cfg['home_joint_angles'])
                    await self.arm.return_to_home(p2_cfg['return_speed'])
                    await asyncio.sleep(1.0)

                    # ==================== Phase 2：被动柔顺 ====================
                    logger.info("\n" + "="*30 + " 【Phase 2：连续柔顺跟随】 " + "="*30)
                    await self.execute_phase2(p2_cfg, CFG_PHASE2['collision_detection']['force_threshold'])

                logger.info("\n>>> 🎬 本轮连招展示完毕！3 秒后自动开启下一轮重置...\n")
                await asyncio.sleep(3.0)

        except KeyboardInterrupt:
            logger.info("\n检测到退出指令，正在终止程序...")
        finally:
            await self.cleanup()

    async def execute_phase1(self, cfg, threshold):
        """完美还原 Phase 1 原版的 Depth 碰撞检测逻辑"""
        self.tactile.use_diff = False        # 🌟 强制切换到 Depth 模式
        self.tactile.score_history.clear()   # 清空历史，避免干扰

        logger.info(f"➡️ 开始向前推进 (速度 {cfg['forward_speed']}%)...")
        await self.arm.move_forward_async(cfg['forward_distance'], cfg['forward_speed'])

        # 完美还原原来的校准逻辑
        logger.info(">>> [3/5] 进入运动校准期(2秒)：忽略启动震动，提取平稳前进基准(Depth)...")
        calibration_scores = []
        calib_start = time.time()

        while time.time() - calib_start < 2.0:
            data = self.tactile.collect_one_frame()
            if data is not None:
                if time.time() - calib_start > 1.0:
                    calibration_scores.append(data['force_val'])
                self.tactile.visualize()
            await asyncio.sleep(0.01)

        if len(calibration_scores) > 10:
            self.tactile.baseline_score = np.mean(calibration_scores)
        else:
            self.tactile.baseline_score = 0.0

        self.tactile.score_history.clear()
        # 注意：这里使用 config_needle.yaml 里的低阈值 (如 0.2)
        logger.info(f">>> 动态基准锁定为: {self.tactile.baseline_score:.1f} | 触发阈值: ±{threshold:.1f}")
        logger.info(">>> 监测正式开始，等待气球爆破扰动信号...")

        # 完美还原原来的监测逻辑 (使用 is_collision_detected)
        start_time = time.time()
        last_print = time.time()
        triggered = False

        monitor_t = 30.0
        while time.time() - start_time < monitor_t:
            data = self.tactile.collect_one_frame()
            if data is None:
                await asyncio.sleep(0.01)
                continue

            # 使用原版基于滑动窗口的检测方法
            is_collision = self.tactile.is_collision_detected(data, threshold=threshold)

            if time.time() - last_print > 1.0:
                logger.info(f"持续前进监测中... | 当前受力(Depth): {data['force_val']:.1f}")
                last_print = time.time()

            if not self.tactile.visualize():
                sys.exit(0)

            if is_collision:
                triggered = True
                break

            await asyncio.sleep(0.01)

        if triggered:
            logger.warning("💥 扎中气球！")
            self.arm.stop_move()
            logger.info(f"⬅️ 立即执行战术反弹 {cfg['retreat_distance']*100}cm...")
            await self.arm.emergency_retreat_backward(cfg['retreat_distance'], cfg['retreat_speed'])
            return True
        else:
            logger.warning("⏭️ 未检测到气球，跳过本次连招。")
            return False

    async def execute_phase2(self, cfg, threshold):
        """执行静止校准与柔顺滑动"""
        self.tactile.use_diff = True         # 🌟 强制切换到 Difference 模式
        self.tactile.score_history.clear()

        th_trigger = threshold
        th_release = threshold * 0.5
        ret_speed = cfg['retreat_speed']
        max_dist = cfg.get('max_retreat_distance', 0.30)

        # 1. 静止环境排杂校准
        logger.info(">>> 正在提取静止基准 (等待传感器稳态唤醒)...")
        for _ in range(30):
            self.tactile.collect_one_frame()
            await asyncio.sleep(0.02)

        calib_scores = []
        give_up = time.time() + 5.0
        while len(calib_scores) < 50:
            data = self.tactile.collect_one_frame()
            if data:
                calib_scores.append(data['force_val'])
                self.tactile.visualize()
            if time.time() > give_up: break
            await asyncio.sleep(0.01)

        self.tactile.baseline_score = np.mean(calib_scores) if calib_scores else 0.0
        logger.info(f"🎯 柔顺待命基准锁定: {self.tactile.baseline_score:.1f} | 阈值: ±{th_trigger:.1f}")
        logger.info(">>> 💡 请拿出第二个气球抵住针头！只要保持推力就会连续退让。")

        # 2. 连续柔顺状态机
        start_x = self.arm.get_current_x() or 0.0
        limit_x = start_x + max_dist
        is_yielding = False
        last_print = time.time()

        while True:
            data = self.tactile.collect_one_frame()
            if not data:
                await asyncio.sleep(0.01)
                continue

            curr_f = data['force_val']
            dev = abs(curr_f - self.tactile.baseline_score)
            curr_x = self.arm.get_current_x()
            if not self.tactile.visualize(): sys.exit(0)

            if time.time() - last_print > 0.5:
                status = "🔴 受推力，持续退让中..." if is_yielding else "🟢 待命中，等待推入..."
                logger.info(f"{status} | 偏差: {dev:.1f}")
                last_print = time.time()

            # 抵达全局底线
            if curr_x is not None and curr_x >= limit_x - 0.005:
                logger.info("🛑 已退至 Phase 2 全局安全极限！")
                self.arm.stop_move()
                return

            if not is_yielding:
                if dev > th_trigger:
                    logger.warning("⚠️ 受到持续挤压！开启连续柔顺退让...")
                    self.arm.stop_move()
                    await asyncio.sleep(0.02)
                    await self.arm.start_yielding_to_target(limit_x, ret_speed)
                    is_yielding = True
                else:
                    self.tactile.baseline_score = self.tactile.baseline_score * 0.95 + curr_f * 0.05
            else:
                if dev < th_release:
                    logger.info("✅ 压力减弱或消失，立即刹车！")
                    self.arm.stop_move()
                    is_yielding = False
                    self.tactile.baseline_score = curr_f

            await asyncio.sleep(0.01)

    async def cleanup(self):
        self.arm.stop_move()
        self.tactile.cleanup()
        self.arm.cleanup()
        try:
            from common_imports import libstark
            libstark.modbus_close(self.hand.client)
        except: pass

# ==================== 4. 程序入口 ====================
async def main():
    # 统一提取 IP 和端口
    ip = CFG_PHASE1['arm_control'].get('arm_ip', '192.168.1.19')
    port = CFG_PHASE1['arm_control'].get('arm_port', 8080)
    serial = CFG_PHASE1['tactile'].get('sensor_serial', 'BM000028')

    # 全局仅初始化一次硬件！
    client, slave_id = await open_modbus_revo2()
    hand = RevoHandController(client, slave_id)
    await hand.initialize()

    tactile = SwitchableTactileCollector(serial)
    if not tactile.initialize(): sys.exit(1)

    arm = UnifiedArmController(ip, port)

    # 载入状态机并启动
    controller = MasterDemoController(hand, tactile, arm)
    await controller.run_full_sequence()

if __name__ == "__main__":
    asyncio.run(main())