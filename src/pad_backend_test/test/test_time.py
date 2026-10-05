from types import SimpleNamespace
import pytest
from pad_backend_test.runner import Experiment
from pad_backend_test.simulation_clock import SimulationClock


def test_sequence_uses_ros_clock():
    fake = SimpleNamespace(args=SimpleNamespace(use_sim_time=True),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=5_000_000_000)))
    assert Experiment.sequence_time(fake) == 5.0


def test_wall_mode_uses_monotonic(monkeypatch):
    monkeypatch.setattr('pad_backend_test.runner.time.monotonic', lambda: 123.0)
    assert Experiment.sequence_time(SimpleNamespace(args=SimpleNamespace(use_sim_time=False))) == 123.0


def test_clock_ticks_normalize_seconds():
    messages = []
    fake = SimpleNamespace(stamp=990_000_000, publisher=SimpleNamespace(publish=messages.append))
    SimulationClock.tick(fake)
    assert messages[0].clock.sec == 1
    assert messages[0].clock.nanosec == 0
