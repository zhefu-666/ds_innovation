"""Only logical Open=0 degrees and Close=20 degrees. A6 meaning is configuration data."""
import json
from pathlib import Path
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/frame_20261007.pending.json'
ANGLES = {'open': 0, 'close': 20}

def load_mapping(path):
    data = json.loads(Path(path).read_text())
    if data.get('action_mapping_verified') != 1:
        raise ValueError('A6 / physical action mapping has not been accepted')
    mapping = {name: data.get('a6_' + name + '_state') for name in ANGLES}
    if set(mapping.values()) != {0, 1}:
        raise ValueError('A6 mapping must explicitly assign distinct binary states')
    return mapping

def hold_angle(state, mapping):
    if mapping is None:
        raise ValueError('Cannot infer hold angle without accepted A6 mapping')
    for name, value in mapping.items():
        if value == state:
            return ANGLES[name]
    raise ValueError('Invalid feedback state; cannot preserve frame position')
