#!/usr/bin/env python3
"""Does a trained policy read the subgoal, or just fly at whatever targets it is shown?

A stage can pass on success rate while the policy has never had to tell the
skin target from the subgoal target (stage 0 always shows the same point in
both slots). This check feeds synthetic observations to the exported ONNX
policy -- no Unity needed, a few seconds -- with the skin and the subgoal
point in different (perpendicular) directions, and measures where the action
points.

  python python/evaluation/policy_input_check.py results/stage1_s1/Mosquito.onnx [more.onnx ...]

Reported per subgoal (MOVE_TO, HIDE) and per distance to the point:
  to_point  cosine between the action and the direction to the subgoal point (want ~1)
  to_skin   cosine between the action and the direction to the skin          (want ~0)
Pass (proposed): for MOVE_TO and HIDE, at every point distance from
--pass-from metres up, to_point >= --min-point and |to_skin| <= --max-skin.
Also printed, without a pass rule: how much the action changes when only the
subgoal one-hot changes, and APPROACH with both slots on the skin (the only
form APPROACH takes in the environment).

Free space is assumed (all rays 1, zero velocity, not landed), so this checks
target selection only, not obstacle handling.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Callable, Dict, List

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mosquitosim.obs_spec import INDEX, OBS_SIZE, POS_SCALE  # noqa: E402

SUBGOAL_SLOT = {"APPROACH": 0, "HIDE": 1, "EVADE": 2, "MOVE_TO": 3}
POINT_DISTANCES = (0.15, 0.3, 0.6, 1.0, 2.0)
SKIN_DISTANCES = (0.5, 1.0, 1.5, 2.5)


def make_obs(skin_rel: np.ndarray, point_rel: np.ndarray, subgoal: str) -> np.ndarray:
    """Observations [n, OBS_SIZE] for skin / subgoal-point offsets [n, 3] in metres."""
    n = len(skin_rel)
    o = np.zeros((n, OBS_SIZE), np.float32)

    def put(pos_name: str, dist_name: str, rel: np.ndarray) -> None:
        a, b = INDEX[pos_name]
        o[:, a:b] = np.clip(rel / POS_SCALE, -1.0, 1.0)
        o[:, INDEX[dist_name][0]] = np.minimum(np.linalg.norm(rel, axis=1) / POS_SCALE, 1.0)

    put("target_skin_rel_pos", "target_skin_dist", skin_rel)
    put("subgoal_target_rel_pos", "subgoal_target_dist", point_rel)
    a, b = INDEX["rays"]
    o[:, a:b] = 1.0
    o[:, INDEX["subgoal_onehot"][0] + SUBGOAL_SLOT[subgoal]] = 1.0
    o[:, INDEX["remaining_time"][0]] = 0.8
    return o


def perpendicular_pairs(n: int, rng: np.random.Generator):
    """Random unit vectors u, v [n, 3] with u perpendicular to v."""
    u = rng.normal(size=(n, 3))
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    w = rng.normal(size=(n, 3))
    v = w - (w * u).sum(1, keepdims=True) * u
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    return u, v


def _cos(a: np.ndarray, d: np.ndarray) -> np.ndarray:
    return (a * d).sum(1) / (np.linalg.norm(a, axis=1) + 1e-9)


def check(policy: Callable[[np.ndarray], np.ndarray], n: int = 400, seed: int = 0) -> Dict:
    rng = np.random.default_rng(seed)
    out: Dict = {"per_distance": {}, "onehot_effect": {}, "approach_same_slot": None}
    base = None
    for sg in ("MOVE_TO", "HIDE"):
        rows = []
        for dp in POINT_DISTANCES:
            u, v = perpendicular_pairs(n, rng)
            ds = rng.choice(SKIN_DISTANCES, size=(n, 1))
            skin, point = u * ds, v * dp
            act = np.asarray(policy(make_obs(skin, point, sg)), np.float32)[:, :3]
            rows.append({"point_m": dp, "to_point": float(_cos(act, v).mean()),
                         "to_skin": float(_cos(act, u).mean()),
                         "accel": float(np.linalg.norm(act, axis=1).mean())})
            if dp == 1.0:
                ref = np.asarray(policy(make_obs(skin, point, "APPROACH")), np.float32)[:, :3]
                out["onehot_effect"][sg] = float(np.abs(act - ref).mean())
        out["per_distance"][sg] = rows
    u, _ = perpendicular_pairs(n, rng)
    skin = u * rng.choice(SKIN_DISTANCES, size=(n, 1))
    act = np.asarray(policy(make_obs(skin, skin, "APPROACH")), np.float32)[:, :3]
    out["approach_same_slot"] = float(_cos(act, u).mean())
    return out


def verdict(res: Dict, pass_from: float, min_point: float, max_skin: float) -> Dict[str, bool]:
    ok = {}
    for sg, rows in res["per_distance"].items():
        rows = [r for r in rows if r["point_m"] >= pass_from - 1e-9]
        ok[sg] = all(r["to_point"] >= min_point and abs(r["to_skin"]) <= max_skin for r in rows)
    return ok


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("policies", nargs="+", help="exported ONNX policy files")
    ap.add_argument("--samples", type=int, default=400)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--pass-from", type=float, default=0.3, help="judge point distances from this many metres up")
    ap.add_argument("--min-point", type=float, default=0.8)
    ap.add_argument("--max-skin", type=float, default=0.3)
    ap.add_argument("--json", help="also write the numbers to this file")
    args = ap.parse_args(argv)

    from mosquitosim.onnx_policy import OnnxPolicy

    all_ok, dump = True, {}
    for path in args.policies:
        res = check(OnnxPolicy(path), args.samples, args.seed)
        ok = verdict(res, args.pass_from, args.min_point, args.max_skin)
        res["pass"] = ok
        dump[path] = res
        all_ok &= all(ok.values())
        print(path)
        for sg, rows in res["per_distance"].items():
            cells = "  ".join(f"{r['point_m']:.2f}m {r['to_point']:+.2f}/{r['to_skin']:+.2f}" for r in rows)
            print(f"  {sg:8s} to_point/to_skin by point distance: {cells}  -> {'PASS' if ok[sg] else 'FAIL'}")
        eff = ", ".join(f"{k} {v:.3f}" for k, v in res["onehot_effect"].items())
        print(f"  one-hot effect (mean |action change| vs APPROACH, point at 1 m): {eff}")
        print(f"  APPROACH with both slots on the skin, to_skin: {res['approach_same_slot']:+.2f}")
    if args.json:
        Path(args.json).write_text(json.dumps(dump, indent=1), encoding="utf-8")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
