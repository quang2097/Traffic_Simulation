# Reference network from the supplied train.ipynb; training and Colab cells are excluded.
from typing import Optional, Tuple, List
import numpy as np
import torch
from torch import nn

class TrafficQNetwork(nn.Module):
    def __init__(
        self,
        obs_dim: int = 28,
        act_dim: int = 3,
        hidden_dims: Tuple[int, ...] = (256, 256, 128),
        obs_mean: Optional[np.ndarray] = None,
        obs_std: Optional[np.ndarray] = None,
    ):
        super().__init__()
        mean_t = torch.zeros(obs_dim, dtype=torch.float32) if obs_mean is None else torch.as_tensor(obs_mean, dtype=torch.float32)
        std_t = torch.ones(obs_dim, dtype=torch.float32) if obs_std is None else torch.as_tensor(obs_std, dtype=torch.float32)
        std_t = torch.clamp(std_t, min=1e-6)

        self.register_buffer("obs_mean", mean_t)
        self.register_buffer("obs_std", std_t)

        layers: List[nn.Module] = []
        in_dim = obs_dim
        for h in hidden_dims[:-1]:
            layers.append(nn.Linear(in_dim, h))
            layers.append(nn.LayerNorm(h))
            layers.append(nn.SiLU())
            in_dim = h
        self.backbone = nn.Sequential(*layers)

        last_h = hidden_dims[-1]
        self.val_head = nn.Sequential(
            nn.Linear(in_dim, last_h),
            nn.SiLU(),
            nn.Linear(last_h, 1),
        )
        self.adv_head = nn.Sequential(
            nn.Linear(in_dim, last_h),
            nn.SiLU(),
            nn.Linear(last_h, act_dim),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        x = (obs - self.obs_mean) / self.obs_std
        feat = self.backbone(x)
        val = self.val_head(feat)
        adv = self.adv_head(feat)
        return val + (adv - adv.mean(dim=-1, keepdim=True))
