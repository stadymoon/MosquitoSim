import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "evaluation"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import policy_input_check as pic  # noqa: E402
from mosquitosim.obs_spec import INDEX  # noqa: E402


def _slot(obs, name):
    a, b = INDEX[name]
    return obs[:, a:b]


def follows_subgoal(obs):
    """Reads the subgoal: APPROACH flies at the skin, anything else at the point."""
    approach = obs[:, INDEX["subgoal_onehot"][0]] > 0.5
    return np.where(approach[:, None], _slot(obs, "target_skin_rel_pos"), _slot(obs, "subgoal_target_rel_pos"))


def follows_sum(obs):
    """What stage 0 teaches: fly along the sum of the two targets."""
    return _slot(obs, "target_skin_rel_pos") + _slot(obs, "subgoal_target_rel_pos")


def test_a_policy_that_reads_the_subgoal_passes():
    res = pic.check(follows_subgoal, n=200)
    assert all(pic.verdict(res, 0.3, 0.8, 0.3).values())
    assert res["onehot_effect"]["MOVE_TO"] > 0.05
    assert res["approach_same_slot"] > 0.99


def test_a_policy_that_sums_the_two_targets_fails():
    res = pic.check(follows_sum, n=200)
    assert not any(pic.verdict(res, 0.3, 0.8, 0.3).values())
    assert res["onehot_effect"]["MOVE_TO"] < 1e-6
    near = res["per_distance"]["MOVE_TO"][0]
    assert near["to_skin"] > near["to_point"]      # pulled to the skin next to the point
    assert res["approach_same_slot"] > 0.99        # yet APPROACH looks perfect
