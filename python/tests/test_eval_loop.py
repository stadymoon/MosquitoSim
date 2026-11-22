import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "evaluation"))
from eval_loop import EvalConfig, EvalLoop, episode_index, room_quotas  # noqa: E402
from fake_env import FakeEnv  # noqa: E402
from mosquitosim.latency import LatencyMode  # noqa: E402
from mosquitosim.planner_client import PlanResult  # noqa: E402


def zero_policy(obs):
    return np.zeros((obs.shape[0], 3), np.float32)


def make_planner(text='{"subgoal": "MOVE_TO", "target": "P5"}', service=0.3):
    calls = []

    def plan(instruction, images, use_images):
        calls.append(use_images)
        return PlanResult(text, service, service)
    plan.calls = calls
    return plan


def run(condition, latency="measured", planner=None, episodes=8, rooms=4):
    env = FakeEnv(rooms=rooms)
    cfg = EvalConfig(condition, LatencyMode.parse(latency), episodes=episodes, rooms=rooms)
    res = EvalLoop(env, zero_policy, cfg, planner).run()
    return env, res


def test_quotas_and_ids():
    assert room_quotas(30, 16) == [2] * 14 + [1] * 2
    assert sum(room_quotas(30, 16)) == 30
    assert episode_index("eval_net/s1/r03e0012") == 12


@pytest.mark.parametrize("cond,key", [("rule", "rule_command"), ("fixed", "fixed_command")])
def test_in_unity_commands_apply_without_delay(cond, key):
    env, res = run(cond)
    assert len(res.episodes) == 8
    counted = [d for d in res.decisions]
    assert counted and all(d["delay"] == 0.0 for d in counted)
    for d in counted:
        got = [c for c in env.received if c["episode"] == d["episode"] and c["decision"] == d["decision"]]
        assert got and got[0]["command"] == d[key]
        assert got[0]["t"] - d["requested_at"] <= 0.05 + 1e-9


def test_measured_latency_delays_command_by_service_time():
    planner = make_planner(service=0.3)
    env, res = run("vlm", "measured", planner)
    assert all(planner.calls)  # vlm sends images
    for d in res.decisions:
        assert d["valid"] and d["command"] == {"subgoal": "MOVE_TO", "target": "P5"}
        got = [c for c in env.received if c["episode"] == d["episode"] and c["decision"] == d["decision"]]
        lag = got[0]["t"] - d["requested_at"]
        assert 0.3 - 1e-9 <= lag <= 0.3 + 0.05 + 1e-9


def test_fixed_latency_and_noimg():
    planner = make_planner(service=0.01)
    env, res = run("vlm_noimg", "fixed:1.0", planner)
    assert not any(planner.calls)  # no images for the ablation
    for d in res.decisions:
        assert d["delay"] == 1.0
        got = [c for c in env.received if c["episode"] == d["episode"] and c["decision"] == d["decision"]]
        if got:  # a command due after the episode ended is dropped
            assert got[0]["t"] - d["requested_at"] >= 1.0 - 1e-9


def test_invalid_answer_keeps_previous_command():
    planner = make_planner(text='{"subgoal": "HIDE", "target": "P99"}')
    env, res = run("vlm", "measured", planner)
    assert res.decisions and not any(d["valid"] for d in res.decisions)
    counted = {d["episode"] for d in res.decisions}
    assert not [c for c in env.received if c["episode"] in counted]


def test_pending_commands_dropped_at_episode_end():
    # 2 s latency never fits before the 2 s episode ends at the halfway request
    planner = make_planner(service=1.9)
    env, res = run("vlm", "measured", planner)
    for c in env.received:
        ep_end = [e for e in res.episodes if e["episode"] == c["episode"]]
        if ep_end:
            assert c["t"] <= (episode_index(c["episode"]) + 1) * 40 * 0.05 + 1e-9


def test_realtime_mode_does_not_block():
    planner = make_planner(service=0.0)
    env, res = run("vlm", "realtime", planner, episodes=4)
    assert len(res.episodes) == 4
    assert res.decisions and all(d["valid"] for d in res.decisions)


def test_episode_records_keep_the_room_type():
    env, res = run("fixed")
    assert res.episodes and all(e["room_type"] == "R4x3" for e in res.episodes)
