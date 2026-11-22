"""Evaluation loop: runs the low-level policy, answers planner requests and
injects planner latency in simulated time (docs/interface.md, "평가 루프").

The loop only talks to an EnvAdapter, so it runs the same against Unity
(MLAgentsEnv in evaluate.py) and against the fake environment in the tests.
"""
from __future__ import annotations

import re
import sys
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Protocol, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mosquitosim.channel import command_message  # noqa: E402
from mosquitosim.latency import CommandScheduler, LatencyMode  # noqa: E402
from mosquitosim.planner_client import PlanResult  # noqa: E402
from mosquitosim.subgoals import parse_planner_output  # noqa: E402

CONDITIONS = ("fixed", "rule", "vlm", "vlm_noimg")
_EPISODE_RE = re.compile(r"r(\d+)e(\d+)$")


class EnvAdapter(Protocol):
    def get_obs(self) -> Tuple[np.ndarray, np.ndarray]: ...      # agent ids, obs [n, 59]
    def set_actions(self, agent_ids: np.ndarray, actions: np.ndarray) -> None: ...
    def step(self) -> None: ...
    def messages(self) -> List[dict]: ...
    def send(self, msg: dict) -> None: ...


Planner = Callable[[str, Sequence[str], bool], PlanResult]  # instruction, images, use_images


@dataclass
class EvalConfig:
    condition: str
    latency: LatencyMode
    episodes: int = 30
    rooms: int = 16
    decision_dt: float = 0.05        # simulated seconds per env.step (decision period 5 x 0.01 s)
    max_steps: int = 2_000_000

    def __post_init__(self) -> None:
        if self.condition not in CONDITIONS:
            raise ValueError(f"condition must be one of {CONDITIONS}")


def room_quotas(episodes: int, rooms: int) -> List[int]:
    """Episodes per room: the first (episodes % rooms) rooms get one more."""
    base, extra = divmod(episodes, rooms)
    return [base + (1 if r < extra else 0) for r in range(rooms)]


def episode_index(episode_id: str) -> int:
    m = _EPISODE_RE.search(episode_id)
    if not m:
        raise ValueError(f"bad episode id {episode_id!r}")
    return int(m.group(2))


@dataclass
class EvalResult:
    episodes: List[dict] = field(default_factory=list)
    decisions: List[dict] = field(default_factory=list)
    steps: int = 0
    sim_time: float = 0.0
    wall_time: float = 0.0


class EvalLoop:
    def __init__(self, env: EnvAdapter, policy: Callable[[np.ndarray], np.ndarray], cfg: EvalConfig,
                 planner: Optional[Planner] = None,
                 on_decision: Optional[Callable[[dict], None]] = None,
                 on_episode: Optional[Callable[[dict], None]] = None) -> None:
        if cfg.condition.startswith("vlm") and planner is None:
            raise ValueError("vlm conditions need a planner")
        self.env, self.policy, self.cfg, self.planner = env, policy, cfg, planner
        self.on_decision = on_decision or (lambda d: None)
        self.on_episode = on_episode or (lambda e: None)
        self.quota = room_quotas(cfg.episodes, cfg.rooms)
        self.done_per_room = [0] * cfg.rooms
        self.scheduler = CommandScheduler()
        self.now = 0.0
        self.result = EvalResult()
        self._executor = ThreadPoolExecutor(max_workers=1) if cfg.latency.kind == "realtime" else None
        self._inflight: List[Tuple[Future, dict, float]] = []

    # ------------------------------------------------------------------ main loop
    def run(self) -> EvalResult:
        t0 = time.perf_counter()
        while not self.finished and self.result.steps < self.cfg.max_steps:
            ids, obs = self.env.get_obs()
            if len(ids):
                self.env.set_actions(ids, self.policy(obs))
            for room, episode, decision, cmd in self.scheduler.pop_ready(self.now):
                self.env.send(command_message(room, episode, decision, cmd))
            self.env.step()
            self.result.steps += 1
            self.now += self.cfg.decision_dt
            for msg in self.env.messages():
                self._handle(msg)
            self._poll_realtime()
        if self._executor:
            self._executor.shutdown(wait=False, cancel_futures=True)
        self.result.sim_time = self.now
        self.result.wall_time = time.perf_counter() - t0
        return self.result

    @property
    def finished(self) -> bool:
        return all(d >= q for d, q in zip(self.done_per_room, self.quota))

    # ------------------------------------------------------------------ messages
    def _handle(self, msg: dict) -> None:
        kind = msg.get("type")
        if kind == "episode_end":
            self._episode_end(msg)
        elif kind == "request":
            self._request(msg)

    def _counted(self, msg: dict) -> bool:
        room = int(msg["room"])
        return room < self.cfg.rooms and episode_index(msg["episode"]) < self.quota[room]

    def _episode_end(self, msg: dict) -> None:
        room = int(msg["room"])
        self.scheduler.drop_room(room)
        if not self._counted(msg):
            return
        rec = {k: msg.get(k) for k in ("episode", "room", "room_type", "stage", "success", "caught", "trapped",
                                       "timeout", "sim_time", "collisions", "decisions", "reward")}
        rec["condition"] = self.cfg.condition
        rec["latency"] = str(self.cfg.latency)
        self.result.episodes.append(rec)
        self.done_per_room[room] += 1
        self.on_episode(rec)

    def _request(self, msg: dict) -> None:
        room, episode, decision = int(msg["room"]), msg["episode"], int(msg["decision"])
        if not self._counted(msg):
            # Extra episodes beyond the quota: answer cheaply, do not log.
            self.scheduler.schedule(room, episode, decision, msg["fixed_command"], self.now)
            return

        cond = self.cfg.condition
        log = {
            "episode": episode, "room": room, "decision": decision, "sim_time": msg.get("sim_time"),
            "condition": cond, "latency": str(self.cfg.latency), "events": msg.get("events"),
            "current_command": msg.get("current_command"), "rule_command": msg.get("rule_command"),
            "fixed_command": msg.get("fixed_command"), "skins": msg.get("skins"),
            "candidates": msg.get("candidates"), "images": msg.get("images"),
            "instruction": msg.get("instruction"), "requested_at": self.now,
        }

        if cond in ("fixed", "rule"):
            cmd = msg[f"{cond}_command"]
            log.update(raw=None, command=cmd, valid=True, service_time=0.0, wall_time=0.0, delay=0.0)
            self.scheduler.schedule(room, episode, decision, cmd, self.now)
            self._log(log)
            return

        use_images = cond == "vlm"
        images = msg.get("images") or []
        if self._executor is not None:  # realtime: do not block the simulation
            fut = self._executor.submit(self.planner, msg["instruction"], images, use_images)
            self._inflight.append((fut, log, self.now))
            return
        result = self.planner(msg["instruction"], images, use_images)  # Unity waits meanwhile
        self._finish(log, result, self.cfg.latency.delay(result.service_time))

    def _poll_realtime(self) -> None:
        still = []
        for fut, log, _ in self._inflight:
            if fut.done():
                result = fut.result()
                self._finish(log, result, self.now - log["requested_at"])
            else:
                still.append((fut, log, _))
        self._inflight = still

    def _finish(self, log: dict, result: PlanResult, delay: float) -> None:
        offered_points = [c["id"] for c in (log.get("candidates") or [])]
        offered_skins = [s["id"] for s in (log.get("skins") or [])] or None
        kwargs = {"offered_points": offered_points}
        if offered_skins:
            kwargs["offered_skins"] = offered_skins
        cmd = parse_planner_output(result.text, **kwargs) if not result.error else None
        log.update(raw=result.text, error=result.error, valid=cmd is not None,
                   command=None if cmd is None else {"subgoal": cmd.subgoal.name, **({"target": cmd.target} if cmd.target else {})},
                   service_time=result.service_time, wall_time=result.wall_time, delay=delay)
        if cmd is not None:
            apply_at = log["requested_at"] + delay
            if self.cfg.latency.kind == "realtime":
                apply_at = self.now
            self.scheduler.schedule(log["room"], log["episode"], log["decision"], log["command"], apply_at)
        self._log(log)

    def _log(self, log: dict) -> None:
        self.result.decisions.append(log)
        self.on_decision(log)
