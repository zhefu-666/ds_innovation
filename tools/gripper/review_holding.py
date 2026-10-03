#!/usr/bin/env python3
"""Overlay recorded RKNN detections and a candidate 40-degree holding gate."""
import argparse
import json
from pathlib import Path

import cv2


def accepted(box, region, bottom_y):
    _, _, x, y, width, height = box
    x1, y1, x2, y2 = region
    return x >= x1 and x + width <= x2 and y >= y1 and y + height <= y2 and y + height >= bottom_y


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('detections', type=Path)
    p.add_argument('image_dir', type=Path)
    p.add_argument('output_dir', type=Path)
    args = p.parse_args()
    data = json.loads(args.detections.read_text())
    args.output_dir.mkdir(parents=True, exist_ok=True)
    region = data['candidate_region']
    bottom_y = data['minimum_visible_bottom_y']
    for name, sample in data['samples'].items():
        image = cv2.imread(str(args.image_dir / f'holding_40_{name}.png'))
        if image is None or list(image.shape[1::-1]) != data['image_size']:
            raise ValueError(f'Missing or wrong-sized image: {name}')
        cv2.rectangle(image, tuple(region[:2]), tuple(region[2:]), (0, 255, 255), 2)
        cv2.line(image, (region[0], bottom_y), (region[2], bottom_y), (0, 255, 255), 2)
        count = 0
        for box in sample['detections']:
            label, score, x, y, width, height = box
            ok = accepted(box, region, bottom_y) and score >= .25
            count += ok
            color = (0, 200, 0) if ok else (0, 0, 220)
            cv2.rectangle(image, (x, y), (x + width, y + height), color, 2)
            cv2.putText(image, f'{label} {score:.2f} {"IN" if ok else "OUT"}',
                        (x, max(18, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
        cv2.putText(image, f'{name}: {sample["truth"]}, static gate count={count}',
                    (15, 35), cv2.FONT_HERSHEY_SIMPLEX, .75, (255, 255, 255), 2)
        out = args.output_dir / f'{name}_overlay.png'
        if not cv2.imwrite(str(out), image):
            raise OSError(out)
        print(f'{name}: truth={sample["truth"]} static_gate_count={count} {out}')


if __name__ == '__main__':
    main()
