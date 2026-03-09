"""
触觉仿生手集成演示 - 配置文件模板

复制此文件为 tactile_grasp_config.py，修改参数。
然后在 revo2_tactile_grasp_demo.py 中引入使用。
"""

# ==================== 设备配置 ====================
# 手部设备
HAND_SERIAL_PORT = None  # 手的串口号，None 表示自动检测
                         # 示例: "/dev/ttyUSB0" 或 "COM3"

# 触觉传感器
TACTILE_SENSOR_SERIAL = None  # 触觉传感器序列号，None 表示自动检测
                              # 示例: "BM000026"


# ==================== 触觉采样配置 ====================
# 采样帧率
TACTILE_TARGET_FPS = 30

# 是否显示触觉热力图
DISPLAY_TACTILE_HEATMAP = False

# 力数据平滑处理
# 使用移动平均来减少噪声
TACTILE_SMOOTHING_WINDOW = 3  # 平均最近 N 帧


# ==================== 碰撞检测配置 ====================
# 触觉力阈值（单位：牛顿）
# 当触觉力超过此值时，判定为发生碰撞/接触
FORCE_THRESHOLD = 5.0

# 推荐参数范围：
# - 非常敏感（轻微接触）：2.0 - 3.0 N
# - 中等敏感（正常使用）：4.0 - 6.0 N  
# - 迟钝（需要较大力）：7.0 - 10.0 N
# - 非常迟钝（极端情况）：> 10.0 N

# 调试建议：
# 1. 先用监测模式（--mode monitor）查看未受力时的基准值
# 2. 逐步对物体施加压力，观察力值变化
# 3. 设置阈值为：(基准值 + 目标敏感接触力) / 2


# ==================== 手部抓握控制配置 ====================
# 抓握动作参数
GRASP_POSITION = 800          # 抓握位置（0-1000）
GRASP_DURATION = 500          # 抓握持续时间（毫秒）
GRASP_SPEED = 500             # 抓握速度

# 释放动作参数
RELEASE_POSITION = 0          # 释放位置（0-1000）
RELEASE_DURATION = 300        # 释放持续时间（毫秒）

# 参数说明：
# - POSITION: 目标位置，0=完全张开，1000=完全闭合
#   - 0-300: 轻轻握住（用于脆弱物体）
#   - 400-600: 中等握力
#   - 700-1000: 强握力（用于滑动物体）
#
# - DURATION: 执行动作的时间，单位毫秒
#   - 200-400: 快速动作（手指快）
#   - 500-800: 中等速度（平衡）
#   - 800+: 缓慢动作（精细操作）


# ==================== 演示循环配置 ====================
# 抓握-释放循环间隔
CYCLE_INTERVAL = 3.0          # 释放后等待多少秒再重新抓握（秒）

# 监测超时时间
MONITOR_TIMEOUT = 30.0        # 如果超过此时间未检测到碰撞，强制释放（秒）

# 统计信息打印间隔
PRINT_INTERVAL = 1.0          # 每隔多少秒打印一次统计信息（秒）


# ==================== 高级参数 ====================
# 运行模式
# - "grasp_release": 自动抓握-监测-释放循环
# - "monitor": 仅监测，不执行任何手部动作（用于调试）
RUN_MODE = "grasp_release"

# 日志级别
# "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"
LOG_LEVEL = "INFO"

# 启用数据记录
ENABLE_DATA_LOGGING = False   # 是否保存采集的数据到文件
DATA_LOG_DIR = "./logs"       # 数据日志目录


# ==================== 物体和应用特定配置 ====================
# 根据不同应用场景调整以下参数

# 应用场景 1: 握住易碎物体（如鸡蛋）
FRAGILE_OBJECT_CONFIG = {
    'GRASP_POSITION': 500,
    'GRASP_DURATION': 600,
    'FORCE_THRESHOLD': 3.0,
    'SMOOTHING_WINDOW': 5,
}

# 应用场景 2: 握住光滑物体（如水瓶）
SMOOTH_OBJECT_CONFIG = {
    'GRASP_POSITION': 900,
    'GRASP_DURATION': 400,
    'FORCE_THRESHOLD': 4.0,
    'SMOOTHING_WINDOW': 3,
}

# 应用场景 3: 握住粗糙物体（如砖块）
ROUGH_OBJECT_CONFIG = {
    'GRASP_POSITION': 700,
    'GRASP_DURATION': 300,
    'FORCE_THRESHOLD': 6.0,
    'SMOOTHING_WINDOW': 2,
}

# 使用预设配置示例：
# 在 revo2_tactile_grasp_demo.py 中导入：
# from tactile_grasp_config import FRAGILE_OBJECT_CONFIG
# 然后在 Config 类中应用：
# Config.GRASP_POSITION = FRAGILE_OBJECT_CONFIG['GRASP_POSITION']
# Config.FORCE_THRESHOLD = FRAGILE_OBJECT_CONFIG['FORCE_THRESHOLD']


# ==================== 实验参数 ====================
# 以下参数用于性能测试和实验

# 单次实验的循环次数
EXPERIMENT_CYCLES = 10

# 重复实验的次数
EXPERIMENT_REPEATS = 1

# 实验间隔（秒）
EXPERIMENT_INTERVAL = 5.0


if __name__ == "__main__":
    print("这是一个配置文件模板")
    print("使用方法：")
    print("1. 复制此文件为 tactile_grasp_config.py")
    print("2. 修改参数值")
    print("3. 在 revo2_tactile_grasp_demo.py 中引入配置")
