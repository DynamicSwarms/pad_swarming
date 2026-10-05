import pytest
from pad_backend_test.sequence import validate


@pytest.mark.parametrize('sequence', [[], [{'name': 'bad', 'duration': -1, 'velocity': [0]*4}],
    [{'name': 'bad', 'duration': 1, 'velocity': [float('nan')]*4}],
    [{'name': 'bad', 'duration': 1, 'velocity': [0]*3}]])
def test_reject_invalid(sequence):
    with pytest.raises(ValueError):
        validate(sequence)


def test_valid():
    assert validate([{'name': 'hover', 'duration': 1, 'velocity': [0]*4}])
