#!/usr/bin/env python3
"""Offline 90/120/140 ground overlap calibration. No camera/serial/motion access.

init writes an empty collection form. solve transfers metric ground mapping from
validated UI 90 through paired, undistorted ground points, without solvePnP.
Inputs must be recorded with the chassis stationary and on the same ground plane.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

ANGLES = (90, 120, 140)

def project(h, points):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    q = np.column_stack((points, np.ones(len(points)))) @ h.T
    if not np.isfinite(q).all() or np.any(np.abs(q[:, 2]) < 1e-10):
        raise ValueError('Point projects to infinity')
    return q[:, :2] / q[:, 2:]

def pixels(value, k, d, size, minimum=4):
    p = np.asarray(value, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < minimum or not np.isfinite(p).all():
        raise ValueError('Need at least %d finite [x,y] pixel points' % minimum)
    if np.any(p < 0) or np.any(p[:, 0] >= size[0]) or np.any(p[:, 1] >= size[1]):
        raise ValueError('Pixel outside original image')
    return cv2.undistortPoints(p.reshape(-1, 1, 2), k, d, P=k).reshape(-1, 2)

def base_calibration(path):
    f = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not f.isOpened():
        raise ValueError('Cannot open base calibration')
    k, d = f.getNode('camera_matrix').mat(), f.getNode('dist_coeffs').mat()
    size = (int(f.getNode('image_width').real()), int(f.getNode('image_height').real()))
    if f.getNode('extrinsics_validated').real() != 1 or f.getNode('pitch_model_reference_cdeg').real() != -3000:
        raise ValueError('Base must be the validated UI 90 calibration')
    t = f.getNode('T_camera_from_robot').mat()
    if t is None or t.shape != (4, 4):
        raise ValueError('Missing validated base transform')
    h = np.linalg.inv(k @ t[:3, [0, 1, 3]])
    f.release()
    return k, d, size, h

def transfer(source_h, source_pixels, target_pixels, k, d, size):
    a, b = pixels(source_pixels, k, d, size), pixels(target_pixels, k, d, size)
    if len(a) != len(b):
        raise ValueError('Correspondence counts differ')
    # Fit target->source image transform. Metric scale comes from validated 90.
    pair_h, mask = cv2.findHomography(b, a, cv2.RANSAC, 2.0)
    if pair_h is None or mask is None or int(mask.sum()) < 4:
        raise ValueError('Overlap points are degenerate or inconsistent')
    h = source_h @ pair_h
    if abs(np.linalg.det(h)) < 1e-12:
        raise ValueError('Singular ground mapping')
    residual = np.linalg.norm(project(h, b) - project(source_h, a), axis=1)
    return h, {'pairs':len(a), 'inliers':int(mask.sum()),
               'fit_max_mm':float(residual[mask.ravel().astype(bool)].max()*1000)}

def checks(h, samples, k, d, size):
    p = pixels([x['pixel'] for x in samples], k, d, size, minimum=3)
    expected = np.asarray([x['body_mm'] for x in samples], dtype=np.float64) / 1000
    if expected.shape != p.shape or not np.isfinite(expected).all():
        raise ValueError('Validation needs measured [right_mm,forward_mm] for each point')
    errors = np.linalg.norm(project(h, p) - expected, axis=1)*1000
    return {'count':len(p), 'rms_mm':float(np.sqrt(np.mean(errors**2))),
            'max_mm':float(errors.max()), 'errors_mm':errors.tolist()}

def solve(manifest, base, output, max_error_mm):
    if not np.isfinite(max_error_mm) or max_error_mm <= 0:
        raise ValueError('max-error-mm must be positive and chosen for the stopping budget')
    data = json.loads(Path(manifest).read_text())
    if data.get('stationary_chassis') is not True:
        raise ValueError('Confirm stationary_chassis only after collecting stationary pairs')
    k, d, size, h90 = base_calibration(base)
    if data.get('image_size') != list(size):
        raise ValueError('Image resolution differs from base calibration')
    mappings = {90:h90}
    report = {'method':'ground_overlap_homography_no_pnp', 'base':str(Path(base).resolve()),
              'max_error_mm':max_error_mm, 'pairs':[], 'validation':{}}
    pair_data = data.get('pairs', [])
    if len(pair_data) != 2:
        raise ValueError('Need ordered 90->120 and 120->140 pairs')
    for item, (a,b) in zip(pair_data, ((90,120),(120,140))):
        if (item.get('source_ui_deg'),item.get('target_ui_deg')) != (a,b):
            raise ValueError('Pairs must be ordered 90->120 then 120->140')
        h, summary = transfer(mappings[a], item['source_pixels'],item['target_pixels'],k,d,size)
        mappings[b] = h
        report['pairs'].append(dict(source=a,target=b,**summary))
    for angle in ANGLES:
        samples = data.get('validation',{}).get(str(angle),[])
        report['validation'][str(angle)] = checks(mappings[angle],samples,k,d,size)
    output = Path(output)
    if output.exists():
        raise ValueError('Output directory already exists')
    output.mkdir(parents=True)
    passed = all(v['max_mm'] <= max_error_mm for v in report['validation'].values())
    report['passed'] = passed
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if not passed:
        raise ValueError('Independent ruler-point validation failed; see report.json; no YAML emitted')
    # Keep measured 90-degree extrinsics for elevated-plane operations at 90 only.
    # Runtime uses H directly for ground distance and its decomposition for
    # existing object-height checks; no solvePnP is used.
    base_text = Path(base).read_text()
    base_text = '\n'.join(line for line in base_text.splitlines()
                            if not line.startswith('enable_pnp_crosscheck:'))+'\n'
    extra = output/'overlap-extra.yaml'
    fs = cv2.FileStorage(str(extra),cv2.FILE_STORAGE_WRITE)
    fs.write('enable_pnp_crosscheck',0)
    fs.startWriteStruct('overlap_ground_views',cv2.FileNode_SEQ)
    for angle,h in mappings.items():
        fs.startWriteStruct('',cv2.FileNode_MAP)
        fs.write('pitch_cdeg',(angle-120)*100)
        fs.write('homography',h)
        fs.endWriteStruct()
    fs.endWriteStruct(); fs.release()
    suffix = extra.read_text().split('---',1)[1]
    (output/'camera-overlap.yaml').write_text(base_text+suffix)
    extra.unlink()
    return report

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command',required=True)
    init = sub.add_parser('init',help='create empty correspondence and validation form')
    init.add_argument('--output',required=True)
    run = sub.add_parser('solve',help='solve and independently validate ground mappings')
    run.add_argument('--input',required=True)
    run.add_argument('--base',default='config/camera.yaml')
    run.add_argument('--output',required=True)
    run.add_argument('--max-error-mm',required=True,type=float)
    args = parser.parse_args()
    if args.command == 'init':
        form = {'stationary_chassis':False,'image_size':[1280,720],
            'instructions':'Use original image pixels for same ground marks. Keep chassis fixed per pair; 6-12 spread-out points recommended. Validation: >=3 different ruler-measured ground points per angle, including near 100 mm stopping area. Do not match object tops. No automatic device actions.',
            'pairs':[{'source_ui_deg':a,'target_ui_deg':b,'source_image':'','target_image':'',
                      'source_pixels':[],'target_pixels':[]} for a,b in ((90,120),(120,140))],
            'validation':{str(a):[] for a in ANGLES},
            'validation_record_example':'Each entry: {"pixel":[x,y],"body_mm":[right,forward]}',
            'holding_ui140_polygon':[], 'measured_stopping_margin_mm':None}
        with Path(args.output).open('x') as f: json.dump(form,f,ensure_ascii=False,indent=2)
        print('Created empty form:',args.output)
    else:
        report=solve(args.input,args.base,args.output,args.max_error_mm)
        print(json.dumps(report,indent=2))
        print('Candidate written; no runtime configuration or hardware changed.')

if __name__ == '__main__':
    try: main()
    except (ValueError,KeyError,cv2.error,np.linalg.LinAlgError) as error:
        raise SystemExit(str(error))
