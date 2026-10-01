#!/usr/bin/env python3
"""Overlay a candidate gripper holding rectangle on an original image; no device access."""
import argparse
from pathlib import Path
import cv2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True)
    parser.add_argument('--area', required=True, help='x1,y1,x2,y2 in original pixels')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        parser.error('Cannot read image')
    try:
        x1, y1, x2, y2 = map(int, args.area.split(','))
    except ValueError:
        parser.error('Area must contain exactly four integer coordinates')
    height, width = image.shape[:2]
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        parser.error('Rectangle must lie inside the original image')
    output = Path(args.output)
    if output.exists():
        parser.error('Output already exists; choose a new name')
    cv2.rectangle(image, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 255), 2)
    cv2.putText(image, f'{width}x{height} area={x1},{y1},{x2},{y2}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
    if not cv2.imwrite(str(output), image):
        parser.error('Cannot write overlay')
    print(f'Saved {output}; visual aid only, not proof that an object is held.')


if __name__ == '__main__':
    main()
