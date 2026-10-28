#!/usr/bin/env python3
"""Evaluate UI 140 class-specific box-centre references offline; no device access.
Uses original detector coordinates. New classes share the ordinary target line;
their supplied front/rear/left/right samples define inclusive axis bounds.
"""
import argparse
import json
import math
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2] / 'config/project.json'


def load_reference(project):
    project = Path(project).resolve()
    config = json.loads(project.read_text())
    path = project.parent.parent / config['frame']['gripper']['image_trigger_reference']
    reference = json.loads(path.read_text())
    if reference.get('schema_version') != 3 or reference.get('feature') != 'bbox_center_xy':
        raise ValueError('Expected version 3 bbox-centre image reference')
    samples = {s['role']: s for s in reference['samples']}
    if len(samples) != 3 or set(samples) != {'front', 'correct', 'rear'}:
        raise ValueError('Need front, correct and rear samples')
    bounds = reference['reference_center_px']
    if not 0 < bounds['front'][1] < bounds['correct'][1] < bounds['rear'][1] < reference['image_size'][1]:
        raise ValueError('Reference boundaries must increase from front to rear')
    for role, sample in samples.items():
        x, y, w, h = sample['bbox_xywh_px']
        if [x + w / 2, y + h / 2] != bounds[role]:
            raise ValueError('Reference centre differs from detector box')
    for label, item in reference['classes'].items():
        bounds = item['center_bounds_px']
        if bounds is None:
            continue
        centers = item['reference_center_px']
        expected = [centers['left'][0], centers['front'][1],
                    centers['right'][0], centers['rear'][1]]
        if bounds != expected or not (0 <= bounds[0] < bounds[2] < reference['image_size'][0]
                                     and 0 <= bounds[1] < bounds[3] < reference['image_size'][1]):
            raise ValueError('Class bounds differ from boundary centres')
        for sample in item['samples']:
            x, y, w, h = sample['bbox_xywh_px']
            if [x + w / 2, y + h / 2] != centers[sample['role']]:
                raise ValueError('Class reference centre differs from detector box')
    return reference


def evaluate(reference, bbox, image_size, pitch_ui_deg, label='ordinary_supply'):
    if list(image_size) != reference['image_size']:
        raise ValueError('Use original image resolution; preview pixels are not interchangeable')
    if pitch_ui_deg != reference['camera_ui_deg']:
        raise ValueError('Reference applies only to the recorded camera angle')
    if label not in reference['classes']:
        raise ValueError('No recorded centre reference for this class')
    if len(bbox) != 4 or not all(math.isfinite(v) for v in bbox):
        raise ValueError('Need finite bbox x y width height')
    x, y, width, height = bbox
    if (x < 0 or y < 0 or width <= 0 or height <= 0 or
            x + width > image_size[0] or y + height > image_size[1]):
        raise ValueError('Detection box is outside the original image')
    center_x = x + width / 2
    center_y = y + height / 2
    item = reference['classes'][label]
    bounds = item['reference_center_px']
    front, rear = bounds['front'][1], bounds['rear'][1]
    target_x, target = item['target_center_px']
    rectangle = item['center_bounds_px']
    lateral = None if rectangle is None else rectangle[0] <= center_x <= rectangle[2]
    inside = None if lateral is None else lateral and front <= center_y <= rear
    if center_y < front:
        position = 'before_front'
    elif center_y == front:
        position = 'at_front'
    elif center_y < target:
        position = 'front_to_target'
    elif center_y == target:
        position = 'at_target'
    elif center_y < rear:
        position = 'target_to_rear'
    elif center_y == rear:
        position = 'at_rear'
    else:
        position = 'past_rear'
    return {
        'position': position,
        'bbox_center_px': [center_x, center_y],
        'target_error_y_px': center_y - target,
        'center_error_x_px': center_x - target_x,
        'within_longitudinal_reference': front <= center_y <= rear,
        'target_line_reached': center_y >= target,
        'label': label,
        'lateral_fit': 'unmeasured' if lateral is None else ('inside' if lateral else 'outside'),
        'within_grasp_center_range': inside,
        'center_bounds_px': rectangle,
        'observation_only': True,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', default=str(PROJECT))
    parser.add_argument('--label', default='ordinary_supply')
    parser.add_argument('--bbox', nargs=4, type=float, metavar=('X', 'Y', 'W', 'H'))
    parser.add_argument('--image-size', nargs=2, type=int)
    parser.add_argument('--pitch-ui-deg', type=int)
    args = parser.parse_args()
    reference = load_reference(args.project)
    if args.bbox is not None:
        if args.image_size is None or args.pitch_ui_deg is None:
            parser.error('--bbox requires --image-size and --pitch-ui-deg')
        result = evaluate(reference, args.bbox, args.image_size, args.pitch_ui_deg, args.label)
    else:
        result = {label: {s['role']: evaluate(reference, s['bbox_xywh_px'],
                  reference['image_size'], reference['camera_ui_deg'], label)
                  for s in item['samples']} for label, item in reference['classes'].items()}
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError) as error:
        raise SystemExit(str(error))
