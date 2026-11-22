"""A stand-in for the Unity environment that speaks the planner-channel protocol."""
from __future__ import annotations

from typing import List

import numpy as np

from mosquitosim import obs_spec


class FakeEnv:
    """Each room runs fixed-length episodes and asks the planner at the start
    and halfway. Commands are recorded with the simulated time they arrive."""

    def __init__(self, rooms: int = 4, steps_per_episode: int = 40, dt: float = 0.05,
                 split: str = "eval_familiar") -> None:
        self.rooms, self.spe, self.dt, self.split = rooms, steps_per_episode, dt, split
        self.t = 0.0
        self.k = [0] * rooms
        self.steps = [0] * rooms
        self.decision = [0] * rooms
        self.outbox: List[dict] = []
        self.received: List[dict] = []
        self.actions_seen = 0
        for r in range(rooms):
            self._request(r, ["episode_start"])

    def episode_id(self, r: int) -> str:
        return f"{self.split}/s1/r{r:02d}e{self.k[r]:04d}"

    def _request(self, r: int, events) -> None:
        self.outbox.append({
            "type": "request", "room": r, "episode": self.episode_id(r), "decision": self.decision[r],
            "sim_time": self.t, "stage": 2, "events": events,
            "current_command": {"subgoal": "APPROACH", "target": "S1"},
            "skins": [{"id": f"S{i}", "name": "x", "dist": 1.0 + i, "covered": False} for i in range(1, 6)],
            "candidates": [{"id": f"P{i}", "pos": [0, 1, 0], "dist": 0.5 * i, "hidden": i == 2,
                            "clear_path": True, "near_trap": False} for i in range(1, 9)],
            "instruction": "피부: S1 머리 2.0m, S2 왼팔 0.9m, S3 오른팔 3.0m\n",
            "images": [],
            "rule_command": {"subgoal": "HIDE", "target": "P2"},
            "fixed_command": {"subgoal": "APPROACH", "target": "S2"},
        })
        self.decision[r] += 1

    # EnvAdapter
    def get_obs(self):
        o = np.zeros((self.rooms, obs_spec.OBS_SIZE), np.float32)
        return np.arange(self.rooms), o

    def set_actions(self, ids, actions):
        assert actions.shape == (self.rooms, 3)
        self.actions_seen += 1

    def send(self, msg: dict) -> None:
        if msg["type"] == "command":
            self.received.append({**msg, "t": self.t})

    def messages(self):
        out, self.outbox = self.outbox, []
        return out

    def step(self) -> None:
        self.t += self.dt
        for r in range(self.rooms):
            self.steps[r] += 1
            if self.steps[r] == self.spe // 2:
                self._request(r, ["alert_start"])
            if self.steps[r] >= self.spe:
                self.outbox.append({"type": "episode_end", "room": r, "room_type": "R4x3",
                                    "episode": self.episode_id(r),
                                    "stage": 2, "success": r % 2 == 0, "caught": r % 2 == 1,
                                    "trapped": False, "timeout": False, "sim_time": self.spe * self.dt,
                                    "collisions": 0, "decisions": self.decision[r], "reward": 0.0})
                self.k[r] += 1
                self.steps[r] = 0
                self.decision[r] = 0
                self._request(r, ["episode_start"])
