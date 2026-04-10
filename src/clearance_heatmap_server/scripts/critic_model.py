#!/usr/bin/env python3
# -*- coding: utf-8 -*-

try:
    import torch.nn as nn
except ImportError as exc:
    raise ImportError("critic_model.py requires PyTorch to be installed.") from exc


class SpatialCriticNet(nn.Module):
    """
    仅供离线训练/实验使用的旧 Critic 网络结构。
    在线热力图服务当前不再依赖该网络。
    """

    def __init__(self, state_dim=3, max_obstacles=10):
        super(SpatialCriticNet, self).__init__()
        self.input_dim = state_dim + max_obstacles * 6
        self.fc1 = nn.Linear(self.input_dim, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, 1)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        return self.fc3(x)
