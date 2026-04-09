"""
触觉仿生手 + 机械臂：【扎破气球 -> 战术反弹 -> 原点复位 -> 柔顺退让】 全新综合连招 Demo
"""
import atexit
import collections
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
        self.use_diff = False  # 默认为 False，代表 Phase 1

        # ====== XYZ 曲线的数据队列 ======
        self.plot_history_len = 150
        self.force_x_hist = collections.deque([0.0]*self.plot_history_len, maxlen=self.plot_history_len)
        self.force_y_hist = collections.deque([0.0]*self.plot_history_len, maxlen=self.plot_history_len)
        self.force_z_hist = collections.deque([0.0]*self.plot_history_len, maxlen=self.plot_history_len)

        # 🌟 修复 2：用于 EMA 平滑滤波的历史变量
        self.smooth_fx = 0.0
        self.smooth_fy = 0.0
        self.smooth_fz = 0.0

    def draw_force_curves(self, target_h):
        """辅助方法：在右侧绘制三行横向 XYZ 曲线图"""
        plot_w = 400  # 右侧曲线图的固定宽度
        # 创建深色背景画板 (BGR: 30,30,30)
        canvas = np.full((target_h, plot_w, 3), 30, dtype=np.uint8)

        # 这里的数据来源假设你类中已有 force_x_hist 等 deque 队列
        # 如果没有，请确保在类的 __init__ 中初始化它们
        hists = [getattr(self, 'force_x_hist', [0]*150),
                 getattr(self, 'force_y_hist', [0]*150),
                 getattr(self, 'force_z_hist', [0]*150)]
        colors = [(0, 0, 255), (0, 255, 0), (255, 100, 100)] # BGR: 红, 绿, 浅蓝
        titles = ['Force X', 'Force Y', 'Force Z']
        strip_h = target_h // 3  # 将画板高度三等分

        for i in range(3):
            y_offset = i * strip_h
            hist = list(hists[i])

            # 画每个条带的边框和标题
            cv2.rectangle(canvas, (0, y_offset), (plot_w-1, y_offset + strip_h - 1), (100, 100, 100), 1)
            cv2.putText(canvas, titles[i], (10, y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 1, cv2.LINE_AA)

            # 缩放逻辑：强制最小跨度为 2.0，防止微小噪声被拉伸成陡峭曲线
            min_v, max_v = min(hist), max(hist)
            rng = max(max_v - min_v, 2.0)

            pts = []
            for x_idx, val in enumerate(hist):
                px = int((x_idx / len(hist)) * plot_w)
                # Y轴坐标映射 (反转Y轴使得正值朝上)
                norm_v = (val - min_v) / rng
                py = y_offset + strip_h - 15 - int(norm_v * (strip_h - 30))
                pts.append((px, py))

            if len(pts) > 1:
                pts_arr = np.array(pts, np.int32).reshape((-1, 1, 2))
                cv2.polylines(canvas, [pts_arr], False, colors[i], 2, cv2.LINE_AA)
        return canvas

    def collect_one_frame(self):
        if not self.sensor: return None
        try:
            # ========================================================
            # 不管原文件用的是 Depth 还是 ImgObjEnhance，直接继承调用
            # ========================================================
            parent_data = super().collect_one_frame()
            if parent_data is None: return None

            # ========================================================
            # 2. 如果处于 Phase 2，拦截得分并替换为 Difference 的分数
            # ========================================================
            if self.use_diff:
                try:
                    raw_diff = self.sensor.selectSensorInfo(Sensor.OutputType.Difference)
                    if raw_diff is not None:
                        diff_gray = cv2.cvtColor(raw_diff, cv2.COLOR_BGR2GRAY)
                        parent_data['force_val'] = float(np.mean(diff_gray))
                except Exception as e:
                    logger.error(f"提取 Difference 异常: {e}")

            # ========================================================
            # 3. 提取 3D 力学数据用于画 XYZ 曲线
            # ========================================================
            fx, fy, fz = 0.0, 0.0, 0.0
            try:
                diff,raw_force = self.sensor.selectSensorInfo(Sensor.OutputType.Difference,Sensor.OutputType.Force3DTuned)
                if raw_force is not None:
                    if len(raw_force.shape) == 3:
                        fx, fy, fz = np.mean(raw_force, axis=(0, 1))
                    else:
                        fx, fy, fz = raw_force[0], raw_force[1], raw_force[2]
            except Exception:
                pass

            # 🌟 修复 2 续：指数移动平均 (EMA) 低通滤波
            # 新值只占 30% 权重，70% 继承老状态，极大滤除了高频毛刺
            self.smooth_fx = 0.7 * self.smooth_fx + 0.3 * fx
            self.smooth_fy = 0.7 * self.smooth_fy + 0.3 * fy
            self.smooth_fz = 0.7 * self.smooth_fz + 0.3 * fz

            self.force_x_hist.append(self.smooth_fx)
            self.force_y_hist.append(self.smooth_fy)
            self.force_z_hist.append(self.smooth_fz)

            # 4. 画面无缝拼接
            if hasattr(self, 'display_img') and self.display_img is not None:
                # 确保底图是 3 通道彩色图 (如果父类吐出来是单通道灰度，自动转换)
                if len(self.display_img.shape) == 2:
                    base_img = cv2.cvtColor(self.display_img, cv2.COLOR_GRAY2BGR)
                else:
                    base_img = self.display_img

                h, w, _ = base_img.shape
                curves_canvas = self.draw_force_curves(target_h=h)

                # 把曲线拼接在右边，并覆盖回 display_img
                self.display_img = np.hstack([base_img, curves_canvas])

            return parent_data

        except Exception as e:
            logger.error(f"采集触觉数据异常: {e}")
            return None

    def cleanup(self):
        """极度安全的清理函数，不管 SDK 内部什么状态都强行释放"""
        try:
            if hasattr(self, 'sensor') and self.sensor is not None:
                # 强行释放底层的 C++ 指针，并忽略它自己的报错
                try:
                    self.sensor.release()
                except Exception:
                    pass
        except Exception:
            pass

        # 调用父类的清理（通常包含关窗口等）
        try:
            super().cleanup()
        except Exception:
            pass

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
                # if True:
                if hit_success:
                    logger.info(">>> [衔接] 扎破气球并已战术后退。保持镜头感 2 秒...")
                    await asyncio.sleep(2.0)

                    # 🌟 核心修改：删除了所有 return_to_home 的代码！
                    logger.info(">>> [衔接] 机械臂原地待命，直接无缝切入 Phase 2...")

                    # 只需要拿一下 Phase 2 的参数传进去即可
                    p2_cfg = CFG_PHASE2['arm_control']
                    # self.arm.set_home_joint(p2_cfg['home_joint_angles'])
                    # await self.arm.return_to_home(p2_cfg['return_speed'])
                    # await asyncio.sleep(1.0)

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

    # 🌟 核心保底机制：注册操作系统的退出钩子
    # 无论程序是怎么死的，系统在回收 Python 进程前都必会执行这个函数！
    def force_release_camera():
        print("\n[系统守护] 监测到程序退出，正在强制释放 Xense 触觉相机资源...")
        try:
            tactile.cleanup()
        except:
            pass

    atexit.register(force_release_camera)

    # 如果这里初始化失败了，退出钩子也能保证不会抛 NoneType 异常
    if not tactile.initialize():
        sys.exit(1)

    arm = UnifiedArmController(ip, port)

    # 载入状态机并启动
    controller = MasterDemoController(hand, tactile, arm)

    try:
        await controller.run_full_sequence()
    except BaseException as e:
        # BaseException 能抓住除了拔电源以外的几乎所有强杀指令（包括 Ctrl+C 和 SystemExit）
        logger.warning(f"\n⚠️ 程序被强制中断或发生严重错误: {e}")
    finally:
        logger.info(">>> 正在执行硬件安全断开流程...")
        try:
            await controller.cleanup()
        except Exception:
            pass

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[系统退出] 用户手动中止 (Ctrl+C)。")