import math


def validate(sequence):
    if not isinstance(sequence, list) or not sequence:
        raise ValueError('Sequence must be a nonempty list')
    for step in sequence:
        if not isinstance(step.get('name'), str) or not step['name']:
            raise ValueError('Each step needs a name')
        duration = step.get('duration')
        if not isinstance(duration, (float, int)) or not math.isfinite(duration) or duration <= 0:
            raise ValueError('Duration must be finite and positive')
        velocity = step.get('velocity', [])
        if len(velocity) != 4 or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in velocity):
            raise ValueError('Velocity must contain finite vx, vy, vz, yaw_rate')
    return sequence
