# Reference model from train (1).ipynb; no training or Colab cells are executed.
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
        min_green: float = 5.0,
    ):
        super().__init__()
        mean_t = torch.zeros(obs_dim, dtype=torch.float32) if obs_mean is None else torch.as_tensor(obs_mean, dtype=torch.float32)
        std_t = torch.ones(obs_dim, dtype=torch.float32) if obs_std is None else torch.as_tensor(obs_std, dtype=torch.float32)
        std_t = torch.clamp(std_t, min=1e-6)

        self.register_buffer("obs_mean", mean_t)
        self.register_buffer("obs_std", std_t)
        self.register_buffer("min_green_t", torch.tensor(float(min_green), dtype=torch.float32))

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

    def compute_action_mask(self, obs: torch.Tensor) -> torch.Tensor:
        """
        Hard Safety Interlock Mask:
          1. If either group is in Yellow (phase == 1) or active Tr timer (tr_remaining_s > 0),
             ONLY Action 2 (Hold) is permitted until Tr finishes.
          2. If a group is Green (phase == 2) with phase_age_s < min_green,
             switching to the opposing group is blocked.
        """
        p0 = torch.round(obs[:, 4])
        age0 = obs[:, 5]
        tr0 = obs[:, 7]

        p1 = torch.round(obs[:, 18])
        age1 = obs[:, 19]
        tr1 = obs[:, 21]

        locked = (p0 == 1) | (p1 == 1) | (tr0 > 0.0) | (tr1 > 0.0)

        can_act0 = (~locked) & ~((p1 == 2) & (age1 < self.min_green_t))
        can_act1 = (~locked) & ~((p0 == 2) & (age0 < self.min_green_t))
        can_act2 = torch.ones_like(locked, dtype=torch.bool)

        return torch.stack([can_act0, can_act1, can_act2], dim=-1)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        x = (obs - self.obs_mean) / self.obs_std
        feat = self.backbone(x)
        val = self.val_head(feat)
        adv = self.adv_head(feat)
        raw_q = val + (adv - adv.mean(dim=-1, keepdim=True))

        mask = self.compute_action_mask(obs)
        neg_inf = torch.full_like(raw_q, -1e9)
        return torch.where(mask, raw_q, neg_inf)
