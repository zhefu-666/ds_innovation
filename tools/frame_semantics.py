"""Only logical Open=0 degrees and Close=20 degrees. A6 meaning is configuration data.

a6_semantics="done_flag" (TEMP_ASSUMPTION 2026-10-08): A6 byte 1 is an action-finished flag that reads 1
after either open or close; it is not a position sensor. The frame angle can then only come from the
last completed command, so holding tools need the operator-known angle and action id of this session.
"""
import json
from pathlib import Path
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/frame_20261007.pending.json'
ANGLES = {'open': 0, 'close': 20}
DONE_FLAG_NOTE = 'TEMP_ASSUMPTION a6_semantics=done_flag: A6=1 only means the id finished; angle is the known/last sent target'


def load_mapping(path):
    data = json.loads(Path(path).read_text())
    if data.get('action_mapping_verified') != 1:
        raise ValueError('A6 / physical action mapping has not been accepted')
    if data.get('a6_semantics') == 'done_flag':
        settle = data.get('a6_done_settle_ms')
        if type(settle) is not int or not 100 <= settle <= 3000:
            raise ValueError('a6_done_settle_ms must be integer 100..3000')
        return {'semantics': 'done_flag', 'settle_ms': settle}
    if data.get('a6_semantics') not in (None, 'state_mapping'):
        raise ValueError('Unknown a6_semantics')
    mapping = {name: data.get('a6_' + name + '_state') for name in ANGLES}
    if set(mapping.values()) != {0, 1}:
        raise ValueError('A6 mapping must explicitly assign distinct binary states')
    return mapping


def done_flag(mapping):
    return isinstance(mapping, dict) and mapping.get('semantics') == 'done_flag'


def hold_angle(state, mapping, known=None):
    """known=(angle, action_id) is required in done_flag mode; state must be the finished flag 1."""
    if mapping is None:
        raise ValueError('Cannot infer hold angle without accepted A6 mapping')
    if done_flag(mapping):
        if known is None or known[0] not in ANGLES.values():
            raise ValueError('Done-flag A6 cannot tell open from close; pass --known-frame-angle/--known-frame-id')
        if state != 1:
            raise ValueError('Last frame action is not finished (A6 flag != 1); cannot preserve frame position')
        return known[0]
    for name, value in mapping.items():
        if value == state:
            return ANGLES[name]
    raise ValueError('Invalid feedback state; cannot preserve frame position')


def check_known_id(feedback_id, known):
    """Done-flag mode: the MCU id must still be the one the operator recorded."""
    if known is not None and feedback_id != known[1]:
        raise ValueError('A6 action id %d differs from --known-frame-id %d; frame position unknown' % (feedback_id, known[1]))
