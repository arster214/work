#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
train_critic.py
职责：独立隔离的强化学习/监督学习训练脚本。
核心思想：在不同的随机生成的包围盒环境和机械臂交互状态中，进行“探索(Exploration) -> 产生经验池(Replay Buffer) -> TD Error 参数更新”，最后导出训练好的模型权重供服务端推理。
"""

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
import math
from pathlib import Path

# 训练网络与在线几何热力图服务解耦，避免误解在线链路仍依赖 RL 推理
from critic_model import SpatialCriticNet

class DummyEnv:
    """一个简易的打通训练循环的环境推演模拟器"""
    def __init__(self):
        self.max_obs = 10
        self.collision_penalty = -10.0
        self.max_safe_reward = 1.0
        self.alpha = 5.0
    
    def reset(self):
        # 随机生成 3-6 个障碍物
        num_obs = random.randint(3, 6)
        obs_array = np.zeros(self.max_obs * 6, dtype=np.float32)
        for i in range(num_obs):
            # 随机 x,y,z 以及尺寸 w,h,d
            obs_array[i*6 : i*6+6] = [
                random.uniform(-0.8, 0.8), random.uniform(-0.8, 0.8), random.uniform(0.2, 1.0),
                random.uniform(0.1, 0.3), random.uniform(0.1, 0.3), random.uniform(0.1, 0.3)
            ]
        return obs_array

    def get_reward_for_point(self, pt, obs_array):
        """
        环境对当前点给出真实 Reward 反馈（后续可以在这里接入真正跟物理引擎或 OMPL 碰撞检测器的反馈）
        """
        pt = np.array(pt)
        reward = self.max_safe_reward
        
        for i in range(self.max_obs):
            cx, cy, cz, lx, ly, lz = obs_array[i*6:(i+1)*6]
            if lx == 0: continue
            
            dx = np.abs(pt[0] - cx) - lx/2.0
            dy = np.abs(pt[1] - cy) - ly/2.0
            dz = np.abs(pt[2] - cz) - lz/2.0
            dist = math.sqrt(max(dx, 0.0)**2 + max(dy, 0.0)**2 + max(dz, 0.0)**2)
            
            if dx < 0 and dy < 0 and dz < 0:
                return self.collision_penalty

            # 障碍物外使用指数衰减风险场：
            # dist -> 0 时 reward 接近 -1
            # dist -> +inf 时 reward 平滑趋近于 1
            penalty = self.max_safe_reward - 2.0 * math.exp(-self.alpha * dist)
            reward = min(reward, penalty)
                
        return reward

def generate_replay_buffer(env, num_samples=5000):
    states = []
    target_values = []
    
    for _ in range(num_samples):
        obs_array = env.reset()
        # 在环境中随机采样一些空间点
        for p in range(20):
            pt = [random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(0, 1.2)]
            # 获取该点与特定环境组合后的真实 Reward (Target Value)
            val = env.get_reward_for_point(pt, obs_array)
            state = np.concatenate([pt, obs_array])
            
            states.append(state)
            target_values.append([val])
            
    return torch.tensor(np.array(states), dtype=torch.float32), torch.tensor(np.array(target_values), dtype=torch.float32)

def train():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Start Training on: {device}")
    
    model = SpatialCriticNet(max_obstacles=10).to(device)
    optimizer = optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()
    
    env = DummyEnv()
    
    # RL中的训练循环。这里的 target 可以随着跟别的 Actor/TD 算法交互而动态改变，但基础思路一致。
    epochs = 200
    batch_size = 256
    
    for epoch in range(1, epochs + 1):
        # 1. 探索 / 生成经验
        states_tensor, target_values_tensor = generate_replay_buffer(env, num_samples=100)
        states_tensor, target_values_tensor = states_tensor.to(device), target_values_tensor.to(device)
        
        dataset = torch.utils.data.TensorDataset(states_tensor, target_values_tensor)
        dataloader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)
        
        epoch_loss = 0.0
        for batch_states, batch_targets in dataloader:
            optimizer.zero_grad()
            # 2. 网络前向推断现有的 Q/Value
            preds = model(batch_states)
            # 3. 计算 Temporal Difference 或者 Critic Loss
            loss = loss_fn(preds, batch_targets)
            # 4. 反向传播更新权重
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()
            
        if epoch % 20 == 0:
            print(f"Epoch {epoch}/{epochs} | Avg Loss: {epoch_loss/len(dataloader):.4f}")
            
    # 5. 循环结束，保存模型权重给推理节点使用
    package_root = Path(__file__).resolve().parents[1]
    model_path = package_root / 'models' / 'critic_best.pth'
    model_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(model.state_dict(), str(model_path))
    print(f"Training finished. Model saved to {model_path}")

if __name__ == '__main__':
    train()
