"""Keep the historical task values Open=0 and Close=20 internally.

TX uses signed gripper_offset (+5=open, -5=closed); A6 semantics come from configuration.
The feedback mapping is explicit configuration and cannot be inferred from TX.

A6 state mapping is a configured protocol assumption, not measured physical position.
Done-flag profiles check protocol acknowledgement, not physical position.
"""
import json
import hashlib
import os
import tempfile
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / 'config'))
from project import load_reference
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/project.json#/frame/pending'
ANGLES = {'open': 0, 'close': 20}
DONE_FLAG_NOTE = 'A6状态1按动作应答解释，不区分开关；等待时间取配置，不是机械到位检测'


def state_record_path(port):
    """Share acknowledged gripper targets across tools for this user and serial port."""
    key = hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / ('ds-frame-%d-%s.json' % (os.getuid(), key))


def load_mapping(path):
    data = load_reference(path, Path(__file__).resolve().parents[1])
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


def hold_angle(state, mapping, known=None, action_id=None):
    """Resolve the logical target from a configured A6 state mapping."""
    if mapping is None:
        raise ValueError('Cannot infer hold angle without accepted A6 mapping')
    # 编号0表示下位机上电后未收到任何动作：此时状态字节是上电默认值，不能按开闭映射解释。
    if action_id == 0:
        raise ValueError('A6 action id 0 means the MCU has received no action since power-up; '
                         'state byte cannot be mapped to open/close; move the frame once, then retry')
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
