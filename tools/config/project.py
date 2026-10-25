"""Read the single project configuration and JSON-pointer references."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT / 'config' / 'project.json'

def _unique(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError('Duplicate JSON key: ' + key)
        out[key] = value
    return out

def split_reference(reference, base=ROOT):
    raw = str(reference)
    file, marker, fragment = raw.partition('#')
    path = Path(file)
    if not path.is_absolute():
        path = (Path(base) / path).resolve()
    return path, ('#' + fragment if marker else '')

def _pointer(value, fragment, reference):
    if not fragment:
        return value
    if not fragment.startswith('#/'):
        raise ValueError('Configuration fragment must use #/path: ' + reference)
    current = value
    for part in fragment[2:].split('/'):
        part = part.replace('~1', '/').replace('~0', '~')
        if isinstance(current, list):
            current = current[int(part)]
        elif isinstance(current, dict) and part in current:
            current = current[part]
        else:
            raise ValueError('Missing configuration fragment: ' + reference)
    return current

def load_reference(reference, base=ROOT):
    path, fragment = split_reference(reference, base)
    value = json.loads(path.read_text(), object_pairs_hook=_unique)
    return _pointer(value, fragment, str(reference))

def reference_path(reference, base=ROOT):
    path, fragment = split_reference(reference, base)
    return str(path) + fragment

def reference_exists(reference, base=ROOT):
    path, fragment = split_reference(reference, base)
    if not path.is_file():
        return False
    if not fragment:
        return True
    try:
        load_reference(reference, base)
    except (OSError, ValueError, json.JSONDecodeError, TypeError):
        return False
    return True
