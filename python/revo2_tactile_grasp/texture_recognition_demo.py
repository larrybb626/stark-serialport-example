"""
视触觉传感器纹理识别 Demo

实现两类方案：
1) 传统 CV 特征工程 + 机器学习（统计特征 + 2D Haar/DB1 小波 + SVM/ANN）
2) 端到端 MLP（64x64 灰度图 Flatten -> 256 -> 128 -> 128 -> num_classes）
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import joblib
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from sklearn.neural_network import MLPClassifier
from sklearn.svm import SVC
from torch.utils.data import DataLoader, Dataset


def _load_grayscale_image(image_path: str) -> np.ndarray:
    """加载灰度图为 uint8 2D 数组。优先 PIL，失败时回退到 OpenCV。"""
    try:
        from PIL import Image

        image = Image.open(image_path).convert("L")
        return np.asarray(image, dtype=np.uint8)
    except Exception:
        try:
            import cv2

            image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f"Failed to read image: {image_path}")
            return image.astype(np.uint8)
        except Exception as exc:
            raise RuntimeError(
                "Unable to load image. Please install Pillow (preferred) or OpenCV."
            ) from exc


def _resize_to_tensor(image: np.ndarray, image_size: Tuple[int, int]) -> torch.Tensor:
    """将灰度图 resize 到目标尺寸，并转成 [1, H, W] 的 float Tensor(0~1)。"""
    tensor = torch.from_numpy(image).float().unsqueeze(0).unsqueeze(0) / 255.0
    resized = F.interpolate(
        tensor,
        size=image_size,
        mode="bilinear",
        align_corners=False,
    )
    return resized.squeeze(0)


def _resolve_image_path(index_path: str, image_path: str) -> str:
    if os.path.isabs(image_path):
        return image_path
    return str((Path(index_path).parent / image_path).resolve())


class TactileTextureDataset(Dataset):
    """从 dataset_index.json 读取 image-label 对并按 split 加载。"""

    def __init__(
        self,
        json_file: str,
        split: str = "train",
        image_size: Tuple[int, int] = (64, 64),
    ):
        with open(json_file, "r", encoding="utf-8") as f:
            full_data = json.load(f)

        self.index_file = json_file
        self.data = [item for item in full_data if item.get("split") == split]
        self.image_size = image_size

    def __len__(self) -> int:
        return len(self.data)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        item = self.data[idx]
        image_path = _resolve_image_path(self.index_file, item["image_path"])
        label = int(item["label_id"])
        image = _load_grayscale_image(image_path)
        image_tensor = _resize_to_tensor(image, self.image_size)
        return image_tensor, torch.tensor(label, dtype=torch.long)


def create_dataloader(
    json_file: str,
    split: str,
    image_size: Tuple[int, int] = (64, 64),
    batch_size: int = 32,
    shuffle: bool = True,
) -> DataLoader:
    dataset = TactileTextureDataset(json_file=json_file, split=split, image_size=image_size)
    return DataLoader(dataset, batch_size=batch_size, shuffle=shuffle)


def _safe_skew(values: np.ndarray) -> float:
    mean = float(values.mean())
    std = float(values.std())
    if std < 1e-12:
        return 0.0
    centered = values - mean
    return float(np.mean((centered / std) ** 3))


def _safe_kurtosis(values: np.ndarray) -> float:
    mean = float(values.mean())
    std = float(values.std())
    if std < 1e-12:
        return 0.0
    centered = values - mean
    return float(np.mean((centered / std) ** 4) - 3.0)


def _haar_dwt2(roi: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    使用 Haar(db1) 做一层二维离散小波变换。
    返回 LL, LH, HL, HH。
    """
    h, w = roi.shape
    h_even = h - (h % 2)
    w_even = w - (w % 2)
    x = roi[:h_even, :w_even].astype(np.float32)

    x00 = x[0::2, 0::2]
    x01 = x[0::2, 1::2]
    x10 = x[1::2, 0::2]
    x11 = x[1::2, 1::2]

    ll = (x00 + x01 + x10 + x11) * 0.5
    lh = (x00 - x01 + x10 - x11) * 0.5
    hl = (x00 + x01 - x10 - x11) * 0.5
    hh = (x00 - x01 - x10 + x11) * 0.5
    return ll, lh, hl, hh


def extract_paper1_features(image_path: str, roi_size: Tuple[int, int] = (128, 128)) -> List[float]:
    """提取论文1适配特征：方差/偏度/峰度 + DWT 能量。"""
    image = _load_grayscale_image(image_path)
    roi = _resize_to_tensor(image, roi_size).squeeze(0).numpy()
    roi = (roi * 255.0).astype(np.float32)

    flattened = roi.reshape(-1)

    f_var = float(np.var(roi))
    f_skew = _safe_skew(flattened)
    f_kurt = _safe_kurtosis(flattened)

    ll, lh, hl, hh = _haar_dwt2(roi)
    f_ca = float(np.mean(ll**2))
    f_ch = float(np.mean(lh**2))
    f_cv = float(np.mean(hl**2))
    f_cd = float(np.mean(hh**2))

    return [f_var, f_skew, f_kurt, f_ca, f_ch, f_cv, f_cd]


def build_paths_and_labels_from_index(json_file: str, split: str) -> Tuple[List[str], List[int]]:
    with open(json_file, "r", encoding="utf-8") as f:
        full_data = json.load(f)

    filtered = [item for item in full_data if item.get("split") == split]
    paths = [_resolve_image_path(json_file, item["image_path"]) for item in filtered]
    labels = [int(item["label_id"]) for item in filtered]
    return paths, labels


def train_paper1_models(
    train_paths: Sequence[str],
    train_labels: Sequence[int],
    svm_path: str = "texture_svm.pkl",
    ann_path: str = "texture_ann.pkl",
) -> Dict[str, Any]:
    """训练并保存 SVM 和极简 ANN。"""
    x_train = [extract_paper1_features(path) for path in train_paths]

    svm = SVC(kernel="rbf")
    svm.fit(x_train, train_labels)
    joblib.dump(svm, svm_path)

    ann = MLPClassifier(
        hidden_layer_sizes=(100, 50),
        activation="relu",
        solver="adam",
        max_iter=500,
        random_state=42,
    )
    ann.fit(x_train, train_labels)
    joblib.dump(ann, ann_path)

    return {"svm_model": svm, "ann_model": ann}


def infer_paper1(image_path: str, model_path: str = "texture_ann.pkl") -> int:
    model = joblib.load(model_path)
    features = extract_paper1_features(image_path)
    return int(model.predict([features])[0])


class Paper2MLP(nn.Module):
    def __init__(self, num_classes: int = 4, image_size: Tuple[int, int] = (64, 64)):
        super().__init__()
        self.image_size = image_size
        flatten_dim = image_size[0] * image_size[1]

        self.network = nn.Sequential(
            nn.Flatten(),
            nn.Linear(flatten_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.network(x)


def evaluate_paper2_mlp(model: nn.Module, data_loader: DataLoader, device: str = "cpu") -> float:
    model.eval()
    total = 0
    correct = 0
    with torch.no_grad():
        for images, labels in data_loader:
            images = images.to(device)
            labels = labels.to(device)
            logits = model(images)
            preds = torch.argmax(logits, dim=1)
            correct += int((preds == labels).sum().item())
            total += int(labels.size(0))
    return float(correct / total) if total > 0 else 0.0


def train_paper2_mlp(
    train_loader: DataLoader,
    val_loader: DataLoader,
    num_classes: int,
    model_path: str = "paper2_mlp_weights.pth",
    epochs: int = 30,
    lr: float = 1e-3,
    device: str = "cpu",
) -> Dict[str, Any]:
    model = Paper2MLP(num_classes=num_classes).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    best_val_acc = 0.0
    best_state = None
    val_acc_history: List[Dict[str, Any]] = []

    for epoch in range(epochs):
        model.train()
        for images, labels in train_loader:
            images = images.to(device)
            labels = labels.to(device)

            optimizer.zero_grad()
            logits = model(images)
            loss = criterion(logits, labels)
            loss.backward()
            optimizer.step()

        val_acc = evaluate_paper2_mlp(model, val_loader, device=device)
        val_acc_history.append({"epoch": epoch + 1, "val_acc": float(val_acc)})
        if val_acc >= best_val_acc:
            best_val_acc = val_acc
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}

    if best_state is not None:
        torch.save(best_state, model_path)
        model.load_state_dict(best_state)
    else:
        torch.save(model.state_dict(), model_path)

    return {"model": model, "best_val_acc": best_val_acc, "val_acc_history": val_acc_history}


def infer_paper2_mlp(
    image_path: str,
    model_path: str = "paper2_mlp_weights.pth",
    num_classes: int = 4,
    image_size: Tuple[int, int] = (64, 64),
    device: str = "cpu",
) -> int:
    image = _load_grayscale_image(image_path)
    image_tensor = _resize_to_tensor(image, image_size).unsqueeze(0).to(device)

    model = Paper2MLP(num_classes=num_classes, image_size=image_size).to(device)
    state_dict = torch.load(model_path, map_location=device, weights_only=True)
    model.load_state_dict(state_dict)
    model.eval()

    with torch.no_grad():
        logits = model(image_tensor)
        prediction = int(torch.argmax(logits, dim=1).item())
    return prediction
