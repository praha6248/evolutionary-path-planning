"""Tests for the Python package (pybind11 bindings + experiment helpers)."""
from pathlib import Path

import numpy as np
import pytest

import robot_ga

MAP = str(Path(__file__).resolve().parents[1] / "maps" / "map1.txt")


@pytest.fixture(scope="module")
def m():
    return robot_ga.Map(MAP)


def test_map_properties(m):
    assert (m.width, m.height) == (60, 40)
    assert m.grid.shape == (40, 60)
    assert m.is_free(*m.start) and m.is_free(*m.goal)


def test_goal_value_straight_line(m):
    value, length, collision = robot_ga.goal_value(np.array([m.start, m.goal]), m)
    assert length == pytest.approx(50.0)
    assert value == pytest.approx(length + 100.0 * collision)


def test_goal_value_rejects_bad_shape(m):
    with pytest.raises(ValueError):
        robot_ga.goal_value(np.zeros((3, 3)), m)


def test_make_parameters_overrides_defaults():
    p = robot_ga.make_parameters(pop_size=40, mutation_prob=0.3)
    assert p.pop_size == 40
    assert p.mutation_prob == pytest.approx(0.3)
    assert p.elite == 2


def test_run_is_deterministic_and_monotonic(m):
    a = robot_ga.run(m, generations=60, seed=5, pop_size=60)
    b = robot_ga.run(m, generations=60, seed=5, pop_size=60)
    assert a["best_value"] == b["best_value"]
    best = a["history"]["best"]
    assert len(best) == 61
    assert np.all(np.diff(best) <= 1e-9)  # elitism: best never gets worse


def test_run_series_summary(m):
    series = robot_ga.run_series(m, seeds=range(3), generations=30, pop_size=40)
    s = robot_ga.summarize(series)
    assert s["runs"] == 3
    assert s["best"] <= s["mean"] <= s["worst"]
    assert 0.0 <= s["collision_free"] <= 1.0
