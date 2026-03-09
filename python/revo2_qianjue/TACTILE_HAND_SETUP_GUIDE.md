# 触觉仿生手集成模块 - 文件说明和使用指南

## 📦 创建的文件列表

### 核心程序文件

| 文件名 | 类型 | 说明 | 主要功能 |
|--------|------|------|--------|
| `revo2_tactile_grasp_demo.py` | Python脚本 | 触觉仿生手完整集成演示 | 三个主要模块的整合：触觉采集、阈值判断、手部控制 |
| `verify_tactile_hand_setup.py` | Python脚本 | 环境验证工具 | 验证依赖、设备连接、配置完整性 |
| `tactile_grasp_config_template.py` | Python脚本 | 配置文件模板 | 参数配置、预设场景、实验参数 |

### 文档文件

| 文件名 | 类型 | 说明 | 内容 |
|--------|------|------|------|
| `TACTILE_GRASP_DEMO.md` | Markdown | 完整使用文档 | 功能介绍、快速开始、参数说明、工作流程、高级功能 |
| `TACTILE_QUICK_REFERENCE.md` | Markdown | 快速参考和故障排除 | 常用命令、快速参考表、详细故障排除、调试工作流 |
| `TACTILE_HAND_SETUP_GUIDE.md` | 本文件 | 文件说明和整体指南 | 文件清单、使用流程、整合说明 |

---

## 🎯 使用流程

### 第 1 步：快速验证环境 (5 分钟)

```bash
cd /home/ubuntu/文档/stark-serialport-example/python/revo2

# 运行环境验证
python verify_tactile_hand_setup.py

# 预期输出：所有项都显示 ✓
```

**如果验证失败**，参考 [TACTILE_QUICK_REFERENCE.md](TACTILE_QUICK_REFERENCE.md) 的"故障排除"部分。

---

### 第 2 步：调试和参数确定 (10-20 分钟)

进入**监测模式**来理解你的系统：

```bash
# 仅监测，不控制手
python revo2_tactile_grasp_demo.py --mode monitor

# 或提示热力图实时显示
python revo2_tactile_grasp_demo.py --mode monitor --display-tactile
```

**在监测模式下**：
1. 观察基准力值（无接触）
2. 用手指施压，观察力值变化
3. 确定合适的阈值（通常在基准值 + 5-10N）

```bash
# 根据观测结果调整阈值
python revo2_tactile_grasp_demo.py --mode monitor --force-threshold 6.0
```

---

### 第 3 步：运行完整演示 (可持续运行)

```bash
# 基础运行：自动检测所有设备
python revo2_tactile_grasp_demo.py

# 带参数运行：自定义敏感度和握力
python revo2_tactile_grasp_demo.py \
    --force-threshold 5.0 \
    --grasp-position 800

# 控制特定设备
python revo2_tactile_grasp_demo.py \
    --hand-port /dev/ttyUSB0 \
    --touch-serial BM000026
```

---

### 第 4 步：根据应用场景调整（可选）

复制配置模板并根据应用场景修改：

```bash
# 复制模板
cp tactile_grasp_config_template.py tactile_grasp_config_my_app.py

# 编辑配置
nano tactile_grasp_config_my_app.py
```

在配置文件中预置了三个场景：
- **易碎物体**：低握力，高敏感度
- **光滑物体**：高握力，中等敏感度
- **粗糙物体**：中握力，低敏感度

---

## 📚 文档速查表

### 我想要...

| 需求 | 查阅文档 | 说明 |
|------|---------|------|
| 快速开始 | [TACTILE_GRASP_DEMO.md](TACTILE_GRASP_DEMO.md) 的"快速开始"部分 | 5分钟上手 |
| 理解工作原理 | [TACTILE_GRASP_DEMO.md](TACTILE_GRASP_DEMO.md) 的"工作流程"部分 | 深入理解三个模块 |
| 自定义参数 | [tactile_grasp_config_template.py](tactile_grasp_config_template.py) | 参数详细说明 |
| 解决问题 | [TACTILE_QUICK_REFERENCE.md](TACTILE_QUICK_REFERENCE.md) 的"故障排除"部分 | 分类故障排除 |
| 常用命令 | [TACTILE_QUICK_REFERENCE.md](TACTILE_QUICK_REFERENCE.md) 的"快速参考"部分 | 常见用法速查 |
| 调试触觉 | [TACTILE_QUICK_REFERENCE.md](TACTILE_QUICK_REFERENCE.md) 的"调试工作流"部分 | 系统化调试步骤 |
| 高级功能 | [TACTILE_GRASP_DEMO.md](TACTILE_GRASP_DEMO.md) 的"高级功能扩展"部分 | 自定义碰撞检测、多阶段控制 |

---

## 🔍 核心模块详解

### 模块 1：数据采集 (TactileDataCollector)

```python
# 采集一帧触觉数据
tactile_data = self.tactile.collect_one_frame()

# 输出内容
{
    'force_3d_tuned': (height, width, 3) 数组,  # 原始力数据
    'force_norm': (height, width) 数组,         # 力的模长
    'max_force': 标量,                           # 该帧最大力
    'mean_force': 标量,                          # 该帧平均力
    'timestamp': 时间戳
}
```

**关键方法**：
- `initialize()` - 初始化传感器
- `collect_one_frame()` - 采集单帧数据
- `is_collision_detected()` - 判断是否碰撞
- `get_force_statistics()` - 获取统计信息
- `visualize()` - 显示热力图

### 模块 2：算法判断 (碰撞检测)

```python
# 基于阈值的碰撞检测
is_collision = self.tactile.is_collision_detected(
    current_data=tactile_data,
    threshold=5.0  # 单位：牛顿
)

# 内部使用移动平均法平滑噪声
smoothed_force = np.mean(force_history[-N:])
collision = smoothed_force > THRESHOLD
```

**改进方向**：
- 可改为基于力变化率的检测（快速碰撞）
- 可改为基于热力图形状的检测（识别接触位置）
- 可改为多阈值检测（轻接触/重接触）

### 模块 3：手部控制 (RevoHandController)

```python
# 抓握动作
await self.hand.grasp(position=800, duration=500)

# 释放动作
await self.hand.release(duration=300)

# 获取马达状态
status = await self.hand.get_motor_status()
```

**位置映射**：
- 0: 完全张开
- 500: 半握状态
- 1000: 完全闭合

---

## ⚙️ 配置要点

### 最关键的三个参数

1. **FORCE_THRESHOLD** (触觉力阈值)
   - 影响：碰撞灵敏度
   - 范围：1.0-15.0 N
   - 调整：用 `--force-threshold` 参数

2. **GRASP_POSITION** (抓握位置)
   - 影响：握力大小
   - 范围：0-1000
   - 调整：用 `--grasp-position` 参数

3. **SMOOTHING_WINDOW** (平滑窗口)
   - 影响：噪声平滑程度
   - 范围：1-10
   - 调整：编辑 `Config` 类

### 工作流程图

```
触觉采集 (30 FPS)
    ↓
计算力的合力 (∥F∥)
    ↓
移动平均平滑
    ↓
与阈值比较
    ↓  
超过阈值？
    ├─ 是 → 触发释放 → 手指张开 (0ms)
    └─ 否 → 继续监测 (下一帧)
```

---

## 🚀 快速命令参考

```bash
# 1. 环境检查（首次运行必做）
python verify_tactile_hand_setup.py

# 2. 调试触觉参数
python revo2_tactile_grasp_demo.py --mode monitor --display-tactile

# 3. 标准运行
python revo2_tactile_grasp_demo.py

# 4. 灵敏调整
python revo2_tactile_grasp_demo.py --force-threshold 3.0  # 增敏
python revo2_tactile_grasp_demo.py --force-threshold 10.0  # 降敏

# 5. 握力调整
python revo2_tactile_grasp_demo.py --grasp-position 500   # 轻握
python revo2_tactile_grasp_demo.py --grasp-position 900   # 紧握

# 6. 完整自定义
python revo2_tactile_grasp_demo.py \
    --hand-port /dev/ttyUSB0 \
    --touch-serial BM000026 \
    --force-threshold 5.0 \
    --grasp-position 800 \
    --display-tactile
```

---

## 🔗 集成点说明

### 与现有项目的集成

**该演示脚本与以下现有模块集成**：

1. **Hand Control** (手部控制)
   - 来自：`revo2_utils.py`（`open_modbus_revo2()`）
   - 使用：Modbus 协议控制 Revo2 手部
   - API：`RevoHandController` 类

2. **Tactile Sensor** (触觉传感器)
   - 来自：Xense SDK (`pip install xensesdk`)
   - 使用：`Sensor.OutputType.Force3DTuned`
   - API：`TactileDataCollector` 类

3. **Common Import** (通用导入)
   - 来自：`common_imports.py`
   - 提供：Logger、libstark 库
   - 用途：日志和设备通信

---

## 📊 性能指标

| 指标 | 值 | 说明 |
|------|-----|------|
| 采样频率 | 30 Hz | 可通过 `TARGET_FPS` 调整 |
| 响应延迟 | <300ms | 包括采集+判断+通信 |
| 热力图帧率 | 15-25 FPS | 显示热力图时 |
| 平滑窗口 | 3 帧 | 约 100ms 延迟 |
| 力值精度 | 0.1 N | 取决于传感器 |

---

## 🎓 学习路径

### 初学者
1. 运行 `verify_tactile_hand_setup.py` ✓
2. 阅读 [TACTILE_GRASP_DEMO.md](TACTILE_GRASP_DEMO.md) 的概述部分
3. 运行 `--mode monitor` 理解触觉数据
4. 运行完整演示

### 中级用户
1. 理解三个核心模块（采集/判断/控制）
2. 根据应用场景调整参数
3. 使用配置文件预设不同场景
4. 观察性能指标和优化

### 高级用户
1. 阅读源代码注释和文档
2. 自定义碰撞检测算法
3. 实现多手指协同
4. 集成到更大的系统

---

## 📞 常见问题速答

**Q: 我该从哪里开始？**  
A: 1) 运行 `verify_tactile_hand_setup.py`，2) 用 `--mode monitor` 调试参数，3) 运行完整脚本

**Q: 如何调整灵敏度？**  
A: 使用 `--force-threshold` 参数，低值更敏感，高值更迟钝

**Q: 手指无法自动释放？**  
A: 1) 检查阈值是否过高，2) 用监测模式查看实际力值，3) 增加平滑窗口

**Q: 如何针对不同物体优化？**  
A: 参考 `tactile_grasp_config_template.py` 的预设场景

**Q: 系统是否支持多个手指独立控制？**  
A: 可以扩展 `RevoHandController`，使用 `set_finger_*` 方法单独控制

---

## 📋 环境需求清单

```
✓ Python 3.7+
✓ NumPy
✓ OpenCV (cv2)
✓ Xense SDK (xensesdk)
✓ libstark（来自项目）
✓ Revo2 Hand 设备
✓ 触觉传感器（Xense）
✓ USB 连接（手和传感器分别连接）
```

---

## 🔧 后续开发建议

1. **数据持久化**：保存触觉数据用于分析
2. **可视化界面**：开发 GUI 用于参数调整
3. **多手指控制**：支持更复杂的抓握策略
4. **强化学习**：智能学习最优参数
5. **ROS 集成**：整合到机器人系统

---

## 📄 文件关联图

```
verify_tactile_hand_setup.py
    ↓ (验证)
revo2_tactile_grasp_demo.py
    ├─ 导入 revo2_utils.py (手部控制)
    ├─ 导入 xensesdk (触觉采集)
    └─ 使用 common_imports.py (日志)

tactile_grasp_config_template.py
    └─ 参考配置

TACTILE_GRASP_DEMO.md
    ├─ 完整文档
    └─ 工作流程说明

TACTILE_QUICK_REFERENCE.md
    ├─ 快速参考
    └─ 故障排除
```

---

**创建日期**：2024 年  
**版本**：1.0  
**维护者**：触觉仿生手集成项目

