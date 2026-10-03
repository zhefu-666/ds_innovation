#!/usr/bin/env python3
"""Derive review-only pitch extrinsics from a measured reference extrinsic.

Assumes a fixed horizontal pitch axis parallel to robot x, with the optical
centre exactly 0.05 m from that axis in the robot yz plane. These assumptions
are stronger than the reported camera-body centre being coplanar with the axis.
"""
import argparse
import json
import math
from pathlib import Path


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
            for i in range(len(a))]


def matvec(a, v):
    return [sum(row[k] * v[k] for k in range(len(v))) for row in a]


def transpose(a):
    return [list(row) for row in zip(*a)]


def derive(source, angle_deg, axis_height_m, optical_radius_m, branch):
    t_ref = source['T_camera_from_robot']
    r_ref = [row[:3] for row in t_ref[:3]]
    c_ref = source['camera_position_robot_m']
    dz = axis_height_m - c_ref[2]
    if abs(dz) >= optical_radius_m:
        raise ValueError('Axis height is incompatible with the assumed optical radius')
    dy = math.sqrt(optical_radius_m ** 2 - dz ** 2)
    # branch=behind: axis lies behind the optical centre in robot +y forward coordinates.
    axis = [c_ref[0], c_ref[1] + (-dy if branch == 'behind' else dy), axis_height_m]
    pivot = matvec(r_ref, [axis[i] - c_ref[i] for i in range(3)])
    delta = math.radians(angle_deg - 5.0)
    co, si = math.cos(delta), math.sin(delta)
    rx = [[1, 0, 0], [0, co, -si], [0, si, co]]
    r_new = matmul(rx, r_ref)
    t_old = [t_ref[i][3] for i in range(3)]
    rotated = matvec(rx, t_old)
    rp = matvec(rx, pivot)
    t_new = [rotated[i] + pivot[i] - rp[i] for i in range(3)]
    centre = matvec(transpose(r_new), [-x for x in t_new])
    matrix = [r_new[i] + [t_new[i]] for i in range(3)] + [[0, 0, 0, 1]]
    return {
        'status': 'mechanical_assumption_candidate', 'validated': False,
        'source_pitch_cdeg': 500, 'target_pitch_cdeg': round(angle_deg * 100),
        'axis_branch': branch, 'axis_robot_m': axis,
        'axis_height_m': axis_height_m, 'assumed_optical_radius_m': optical_radius_m,
        'pitch_pivot_camera_m_at_5deg': pivot,
        'T_camera_from_robot': matrix, 'camera_position_robot_m': centre,
        'limitations': [
            'The 5 cm radius was reported for the camera assembly, not measured for the optical centre.',
            'The pitch axis direction and exact optical-centre offset have not been independently verified.',
            'Do not enable ground ranging at 40 deg or set extrinsics_validated=1 from this derivation.'
        ]
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('reference', type=Path)
    p.add_argument('output_dir', type=Path)
    p.add_argument('--angle-deg', type=float, default=40.0)
    p.add_argument('--axis-height-m', type=float, default=0.18)
    p.add_argument('--optical-radius-m', type=float, default=0.05)
    args = p.parse_args()
    source = json.loads(args.reference.read_text())
    if source.get('validated') is not True or source.get('camera_position_robot_m') is None:
        raise ValueError('Expected independently validated reference extrinsics')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for branch in ('behind', 'ahead'):
        result = derive(source, args.angle_deg, args.axis_height_m, args.optical_radius_m, branch)
        dest = args.output_dir / f'pitch_{result["target_pitch_cdeg"]}_{branch}_candidate.json'
        dest.write_text(json.dumps(result, indent=2) + '\n')
        print(dest)


if __name__ == '__main__':
    main()
