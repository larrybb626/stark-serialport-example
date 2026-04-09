# Revo2 Tactile Grasp Example

Revo2 触觉灵巧手抓握示例

## 简介

基于 **电容式传感器-触觉反馈灵巧手** 的自适应抓握示例 
通过实时采集手指触觉信号、执行触觉感知算法（接触检测、刚度识别），并动态调节抓握电流，实现对不同刚度物体的柔顺抓握与自适应控制。

---

## 系统结构与功能

| 模块文件 | 功能简介 |
|-----------|-----------|
| `main.py` | 系统主入口，负责模块初始化与异步调度，执行示例抓握任务。 |
| `data_reader.py` | 实时采集 Revo2 电容式触觉传感器数据，维护固定长度缓冲区。 |
| `tactile_processor.py` | 对采集的触觉数据进行算法处理，包括接触检、刚度识别。 |
| `grasp_controller.py` | 控制灵巧手，根据触觉反馈动态调节手指电流，实现自适应抓握与释放。 |

---
## 核心控制指令与参数说明

系统的核心控制命令为：

```python
await controller.grasp_with_speed(num_fingers=0)

该指令基于位置与速度控制模式执行抓握或释放动作，通过指定手指数量与动作时间来实现不同类型的抓握。
```
num_fingers 参数含义
| num_fingers 值 | 抓握类型   | 参与手指         | 说明                 |
| ---------------- | ------ | ------------ | ------------------ |
| `0`              | 全部释放 | —            | 所有手指张开，回到初始位置      |
| `2`              | 两指捏 | 拇指 + 食指      | 适合捏取小物体（如泡沫块）     |
| `3`              | 三指捏  | 拇指 + 食指 + 中指 | 稳定捏持中等大小物体（如纸杯） |
| `5`              | 五指握  | 拇指 + 四指      | 完整握拳动作，用于抓握较大物体 （如水瓶）   |

---
## Usage

```shell
cd revo2_tactile_grasp
# 安装依赖
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt
# 运行示例
(py310) ➜  python git:(main) ✗ python main.py
```

---

## 视触觉纹理识别 Demo（离线）

新增 `texture_recognition_demo.py`，提供两种可离线部署的纹理识别流程：

1. **Paper1 适配**：统计特征 + 2D Haar(db1)小波 + `SVM/ANN`
2. **Paper2 适配**：`64x64` 灰度图直接 Flatten 输入 `MLP(256,128,128)`

### 数据索引格式

使用 `dataset_index.json` 管理样本，最小字段如下：

- `image_path`: 图像路径（支持相对 `dataset_index.json` 的路径）
- `label_id`: 数值标签
- `split`: `train` / `val` / `test`

### 典型调用

```python
from texture_recognition_demo import (
    create_dataloader,
    build_paths_and_labels_from_index,
    train_paper1_models,
    infer_paper1,
    train_paper2_mlp,
    infer_paper2_mlp,
)

# ---- Paper1: 特征工程 + 机器学习 ----
train_paths, train_labels = build_paths_and_labels_from_index("dataset_index.json", "train")
train_paper1_models(train_paths, train_labels)
pred = infer_paper1("./demo.png", model_path="texture_ann.pkl")

# ---- Paper2: 端到端 MLP ----
train_loader = create_dataloader("dataset_index.json", split="train", batch_size=32, shuffle=True)
val_loader = create_dataloader("dataset_index.json", split="val", batch_size=32, shuffle=False)
train_paper2_mlp(train_loader, val_loader, num_classes=4, model_path="paper2_mlp_weights.pth")
pred2 = infer_paper2_mlp("./demo.png", model_path="paper2_mlp_weights.pth", num_classes=4)
```
