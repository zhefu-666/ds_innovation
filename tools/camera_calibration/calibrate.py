#!/usr/bin/env python3
"""Chessboard intrinsics and measured-point ground calibration (OpenCV + NumPy)."""
import argparse
import glob
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np


def detect(frame, pattern):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    ok, corners = cv2.findChessboardCorners(gray, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                           (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))


def signature(corners, size):
    p = corners.reshape(-1, 2) / np.array(size)
    # Center, apparent size and orientation distinguish views.
    a, b = p[0], p[-1]
    v = p[1] - p[0]
    return np.array([*p.mean(axis=0), np.linalg.norm(b-a),
                     math.atan2(v[1], v[0]) / math.pi])


def object_grid(pattern, square):
    points = np.zeros((pattern[0]*pattern[1], 3), np.float32)
    points[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)*square
    return points


def solve_intrinsics(corners, size, pattern, square):
    objects = [object_grid(pattern, square) for _ in corners]
    rms, k, d, rs, ts = cv2.calibrateCamera(objects, corners, size, None, None)
    errors = []
    for obj, obs, r, t in zip(objects, corners, rs, ts):
        projected, _ = cv2.projectPoints(obj, r, t, k, d)
        errors.append(float(np.sqrt(np.mean(np.sum((projected-obs)**2, axis=2)))))
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Invalid camera solution')
    return rms, k, d, errors


def write_yaml(path, k, d, size, h=None):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise ValueError(f'Cannot write {path}')
    fs.write('camera_matrix', k)
    fs.write('dist_coeffs', d)
    fs.write('image_width', size[0]); fs.write('image_height', size[1])
    fs.write('ground_coordinates', 'x_right_y_forward_metres')
    fs.write('ground_pixel_domain', 'undistorted_pixels')
    fs.write('imu_tilt_limit_deg', 12.)
    if h is not None:
        fs.write('ground_homography', h)
    fs.release()


def intrinsics(args):
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)  # Never overwrite a previous session.
    samples = out/'samples'; samples.mkdir()
    pattern = (args.cols, args.rows)
    observations, signatures, names = [], [], []
    size = None
    cap = None
    if args.images:
        paths = sorted(glob.glob(args.images))
        if not paths:
            raise ValueError('No input images match the pattern')
        frames = ((p, cv2.imread(p)) for p in paths)
    else:
        source = int(args.camera) if args.camera.isdigit() else args.camera
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if not cap.isOpened():
            raise ValueError('Cannot open camera; stop any other camera consumer first')
        def live():
            start = time.monotonic()
            while time.monotonic()-start < args.timeout:
                ok, frame = cap.read()
                if not ok:
                    raise ValueError('Camera disconnected or failed to return a frame')
                yield 'live', frame
        frames = live()
    last_capture = -float('inf')
    try:
        for name, frame in frames:
            if frame is None:
                print('Unreadable image:', name, flush=True); continue
            current = (frame.shape[1], frame.shape[0])
            if size is None:
                size = current
                print('Actual image size:', size, flush=True)
            if size != current:
                raise ValueError('Mixed image sizes; use the same camera mode throughout')
            corners = detect(frame, pattern)
            status = 'Show complete board; vary position, distance and tilt'
            if corners is not None:
                sig = signature(corners, size)
                diverse = not signatures or min(np.linalg.norm(sig-s) for s in signatures) >= .10
                hull = cv2.convexHull(corners)
                large = cv2.contourArea(hull)/(size[0]*size[1]) > .015
                x,y,w,h = cv2.boundingRect(corners)
                sharp = cv2.Laplacian(cv2.cvtColor(frame[y:y+h,x:x+w],cv2.COLOR_BGR2GRAY),cv2.CV_64F).var() >= args.min_sharpness
                interval = bool(args.images) or time.monotonic()-last_capture >= 1.5
                if diverse and large and sharp and interval:
                    observations.append(corners.copy()); signatures.append(sig)
                    filename = samples/f'{len(observations):03d}.png'
                    if not cv2.imwrite(str(filename), frame):
                        raise ValueError('Could not save accepted frame')
                    names.append(str(filename));last_capture=time.monotonic()
                    print(f'Accepted {len(observations)}/{args.views}: {name}',flush=True)
                status = f'Accepted {len(observations)}/{args.views}; move/tilt board'
                cv2.drawChessboardCorners(frame, pattern, corners, True)
            if args.preview:
                cv2.putText(frame,status,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
                cv2.imshow('Calibration (Q to finish)',frame)
                if cv2.waitKey(1)&255 in (ord('q'),27):
                    break
            if len(observations) >= args.views:
                break
    finally:
        if cap is not None: cap.release()
        if args.preview: cv2.destroyAllWindows()
    if len(observations) < args.min_views:
        raise ValueError(f'Only {len(observations)} diverse views; need {args.min_views}. Accepted images retained for retry.')
    rms,k,d,errors = solve_intrinsics(observations,size,pattern,args.square_mm/1000.)
    # Reject grossly inconsistent frames once, retaining enough independent views.
    limit = max(1.,float(np.median(errors))*2.5)
    kept = [i for i,e in enumerate(errors) if e <= limit]
    rejected = [names[i] for i in range(len(names)) if i not in kept]
    if rejected and len(kept) >= args.min_views:
        observations=[observations[i] for i in kept];names=[names[i] for i in kept]
        rms,k,d,errors=solve_intrinsics(observations,size,pattern,args.square_mm/1000.)
    else:
        rejected=[]
    all_points=np.concatenate(observations).reshape(-1,2)
    span=np.ptp(all_points,axis=0)/np.array(size)
    warnings=[]
    if rms > .5: warnings.append('RMS exceeds 0.5 px: inspect images and independent measurements')
    if span.min() < .6: warnings.append('Board corners cover less than 60% of an image dimension')
    if max(errors)>1.: warnings.append('At least one frame exceeds 1 px error')
    report=dict(image_size=size,pattern=pattern,square_mm=args.square_mm,rms_px=rms,
        per_image_rms=dict(zip(names,errors)),rejected_images=rejected,coverage_span=span.tolist(),
        warnings=warnings,status='candidate_requires_validation')
    (out/'report.json').write_text(json.dumps(report,indent=2))
    write_yaml(out/'intrinsics.yaml',k,d,size)
    print(f'Saved {out}/intrinsics.yaml; RMS={rms:.3f} px; views={len(observations)}')
    for warning in warnings: print('WARNING:',warning)
    print('This is an intrinsics candidate, not a validated ground/robot calibration.')


def load_intrinsics(path):
    fs=cv2.FileStorage(str(path),cv2.FILE_STORAGE_READ)
    if not fs.isOpened(): raise ValueError('Cannot open intrinsics file')
    k=fs.getNode('camera_matrix').mat();d=fs.getNode('dist_coeffs').mat()
    size=(int(fs.getNode('image_width').real()),int(fs.getNode('image_height').real()))
    fs.release()
    if k is None or d is None or min(size)<=0:raise ValueError('Invalid intrinsics file or missing image dimensions')
    return k,d,size


def fit_ground(k,d,pixels,world):
    und=cv2.undistortPoints(np.asarray(pixels,np.float32).reshape(-1,1,2),k,d,P=k).reshape(-1,2)
    world=np.asarray(world,np.float32)
    if len(und)<4 or not np.isfinite(und).all() or not np.isfinite(world).all():
        raise ValueError('Need at least four finite correspondences')
    if np.linalg.matrix_rank(world-world.mean(axis=0))<2:
        raise ValueError('Ground points must not be collinear')
    h,mask=cv2.findHomography(und,world,cv2.RANSAC,.015)
    if h is None or mask.sum()<4 or not np.isfinite(h).all():raise ValueError('Ground fit failed')
    return h,mask.reshape(-1).astype(bool)


def ground(args):
    k,d,size=load_intrinsics(args.intrinsics)
    data=json.loads(Path(args.points).read_text())
    if tuple(data['image_size'])!=size:raise ValueError('Point-image resolution differs from intrinsics')
    train=data['fit'];checks=data['check']
    if len(train)<6 or len(checks)<3:raise ValueError('Use >=6 fit points and >=3 independent check points')
    for group in (train,checks):
        for p in group:
            if len(p['pixel'])!=2 or len(p['world_m'])!=2 or not np.isfinite(p['pixel']+p['world_m']).all():
                raise ValueError('Invalid point')
            x,y=p['pixel']
            if not (0<=x<size[0] and 0<=y<size[1]):raise ValueError('Pixel outside calibrated image')
    if {tuple(p['pixel']) for p in train}&{tuple(p['pixel']) for p in checks}:
        raise ValueError('Check points must be independent of fit points')
    h,inliers=fit_ground(k,d,[p['pixel'] for p in train],[p['world_m'] for p in train])
    pixels=np.array([p['pixel'] for p in checks],np.float32).reshape(-1,1,2)
    und=cv2.undistortPoints(pixels,k,d,P=k)
    predicted=cv2.perspectiveTransform(und,h).reshape(-1,2)
    errors=np.linalg.norm(predicted-np.array([p['world_m'] for p in checks]),axis=1)
    passed=bool(np.isfinite(errors).all() and errors.max()<=args.max_error_mm/1000. and inliers.sum()>=6)
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    report=dict(check_errors_mm=(errors*1000).tolist(),inliers=inliers.tolist(),passed=passed,
        max_allowed_mm=args.max_error_mm,valid_region='Only measured ground region; no extrapolation guarantee')
    (out/'ground_report.json').write_text(json.dumps(report,indent=2))
    if not passed:raise ValueError(f'Ground validation failed; report in {out}; camera.yaml not written')
    write_yaml(out/'camera.yaml',k,d,size,h)
    print(f'Saved {out}/camera.yaml; independent maximum error {errors.max()*1000:.1f} mm')


def pick(args):
    image=cv2.imread(args.image)
    if image is None:raise ValueError('Cannot read image')
    data=json.loads(Path(args.world).read_text())
    pending=[(group,p) for group in ('fit','check') for p in data[group]]
    selected=[]
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and len(selected)<len(pending):selected.append([x,y])
    cv2.namedWindow('Ground points',cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('Ground points',click)
    try:
        while True:
            frame=image.copy()
            for i,point in enumerate(selected):
                cv2.circle(frame,tuple(point),4,(0,0,255),-1)
                cv2.putText(frame,str(i+1),tuple(point),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0),1)
            if len(selected)<len(pending):
                group,p=pending[len(selected)]
                message=f'Click {len(selected)+1}: {group} {p["world_m"]} m; U undo, Q cancel'
            else:message='Enter saves; U undo; Q cancels'
            cv2.putText(frame,message,(5,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
            cv2.imshow('Ground points',frame);key=cv2.waitKey(30)&255
            if key in (ord('q'),27):return
            if key==ord('u') and selected:selected.pop()
            if key in (10,13) and len(selected)==len(pending):break
        result={'image_size':[image.shape[1],image.shape[0]],'fit':[],'check':[]}
        for (group,p),pixel in zip(pending,selected):result[group].append(dict(world_m=p['world_m'],pixel=pixel))
        with open(args.output,'x') as f:json.dump(result,f,indent=2)
        print('Saved',args.output)
    finally:cv2.destroyAllWindows()


def snapshot(args):
    if Path(args.output).exists(): raise ValueError('Output already exists')
    source=int(args.camera) if args.camera.isdigit() else args.camera
    cap=cv2.VideoCapture(source,cv2.CAP_V4L2)
    try:
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,args.width);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,args.height)
        if not cap.isOpened():raise ValueError('Cannot open camera')
        for _ in range(15):
            ok,frame=cap.read()
            if not ok:raise ValueError('Camera frame unavailable')
        if not cv2.imwrite(args.output,frame):raise ValueError('Cannot save image')
        print('Saved',args.output,'actual size',frame.shape[1],frame.shape[0])
    finally:cap.release()



def solve_board_pose(k, d, corners, pattern, square, origin, yaw_deg, height=0.):
    """Return transforms with explicit column-vector source/destination conventions."""
    obj = object_grid(pattern, square)
    result = cv2.solvePnPGeneric(obj, corners, k, d, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for rv, tv in zip(result[1], result[2]):
        rotation = cv2.Rodrigues(rv)[0]
        if np.min((rotation @ obj.T + tv.reshape(3, 1))[2]) <= 0:
            continue
        projected, _ = cv2.projectPoints(obj, rv, tv, k, d)
        rms = float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=2))))
        candidates.append((rms, rv, tv))
    if not candidates:
        raise ValueError('No positive-depth planar pose solution')
    candidates.sort(key=lambda v: v[0])
    rms, rv, tv = candidates[0]
    camera_from_board = np.eye(4)
    camera_from_board[:3, :3] = cv2.Rodrigues(rv)[0]
    camera_from_board[:3, 3] = tv.reshape(3)
    angle = math.radians(yaw_deg)
    robot_from_board = np.array([[math.cos(angle), -math.sin(angle), 0, origin[0]],
        [math.sin(angle), math.cos(angle), 0, origin[1]], [0, 0, 1, height], [0, 0, 0, 1.]])
    camera_from_robot = camera_from_board @ np.linalg.inv(robot_from_board)
    robot_from_camera = np.linalg.inv(camera_from_robot)
    # Project the actual robot ground z=0, not the elevated chessboard surface.
    ground_to_pixel = k @ camera_from_robot[:3, [0, 1, 3]]
    if np.linalg.matrix_rank(ground_to_pixel) < 3:
        raise ValueError('Degenerate ground projection')
    h = np.linalg.inv(ground_to_pixel)
    if not np.isfinite(h).all():
        raise ValueError('Nonfinite ground transform')
    if abs(h[2, 2]) > 1e-12:
        h /= h[2, 2]
    return dict(rms_px=rms, planar_candidate_rms_px=[v[0] for v in candidates],
        rvec_board_to_camera=rv.reshape(3).tolist(), tvec_board_to_camera_m=tv.reshape(3).tolist(),
        T_camera_from_board=camera_from_board.tolist(), T_robot_from_board=robot_from_board.tolist(),
        T_camera_from_robot=camera_from_robot.tolist(), T_robot_from_camera=robot_from_camera.tolist(),
        camera_position_robot_m=robot_from_camera[:3, 3].tolist()), h


def extrinsics(args):
    values = [args.square_mm, args.origin_x, args.origin_y, args.yaw_deg,
              args.board_height_mm, args.max_rms, args.max_error_mm]
    if not np.isfinite(values).all() or args.square_mm <= 0 or args.cols < 3 or args.rows < 3 or args.board_height_mm < 0 or args.max_rms <= 0 or args.max_error_mm <= 0:
        raise ValueError('Invalid board, pose or error limits')
    k, d, size = load_intrinsics(args.intrinsics)
    image = cv2.imread(args.image)
    if image is None or (image.shape[1], image.shape[0]) != size:
        raise ValueError('Image missing or resolution differs from intrinsics')
    corners = detect(image, (args.cols, args.rows))
    if corners is None:
        raise ValueError('Complete chessboard not detected')
    grid = corners.reshape(args.rows, args.cols, 2)
    if args.flip_cols: grid = grid[:, ::-1]
    if args.flip_rows: grid = grid[::-1]
    corners = np.ascontiguousarray(grid.reshape(-1, 1, 2))
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    annotated = image.copy()
    for i, point in enumerate(corners[:, 0]):
        pt = tuple(np.round(point).astype(int))
        cv2.circle(annotated, pt, 3, (0, 255, 0), -1)
        cv2.putText(annotated, str(i), pt, cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 0, 255), 1)
    for index, color, text in [(1, (255, 0, 0), '+board X'), (args.cols, (0, 255, 255), '+board Y')]:
        start = tuple(np.round(corners[0, 0]).astype(int)); end = tuple(np.round(corners[index, 0]).astype(int))
        cv2.arrowedLine(annotated, start, end, color, 2)
        cv2.putText(annotated, text, end, cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1)
    if not cv2.imwrite(str(out/'corners_numbered.png'), annotated):
        raise ValueError('Cannot save corner inspection image')
    report, h = solve_board_pose(k, d, corners, (args.cols, args.rows), args.square_mm/1000.,
        (args.origin_x, args.origin_y), args.yaw_deg, args.board_height_mm/1000.)
    report.update(image_size=size, board_pattern=[args.cols, args.rows], square_mm=args.square_mm,
        flip_cols=args.flip_cols, flip_rows=args.flip_rows, corner_order_confirmed=args.confirm_order,
        robot_axes='x right, y forward, z up; metres; right-handed',
        camera_axes='x image right, y image down, z optical forward',
        transform_convention='p_destination = T_destination_from_source @ p_source',
        status='candidate_requires_independent_ground_validation')
    good_pose = report['rms_px'] <= args.max_rms and report['camera_position_robot_m'][2] > 0
    passed = False
    if args.check_points:
        data = json.loads(Path(args.check_points).read_text())
        if tuple(data['image_size']) != size or len(data['check']) < 3:
            raise ValueError('Need >=3 independent ground checks at calibrated resolution')
        pixels = np.array([p['pixel'] for p in data['check']], np.float32).reshape(-1, 1, 2)
        world = np.array([p['world_m'] for p in data['check']], np.float64)
        if world.shape != (len(pixels), 2) or not np.isfinite(world).all() or not np.isfinite(pixels).all():
            raise ValueError('Invalid check coordinates')
        if len(np.unique(pixels[:, 0], axis=0)) < 3 or np.linalg.matrix_rank(world-world.mean(axis=0)) < 2:
            raise ValueError('Independent check points must be distinct and not collinear')
        if (pixels[:, 0] < 0).any() or (pixels[:, 0] >= np.array(size)).any():
            raise ValueError('Check pixels outside image')
        und = cv2.undistortPoints(pixels, k, d, P=k)
        predicted = cv2.perspectiveTransform(und, h).reshape(-1, 2)
        errors = np.linalg.norm(predicted-world, axis=1)*1000
        report['independent_check_errors_mm'] = errors.tolist()
        passed = bool(np.isfinite(errors).all() and max(errors) <= args.max_error_mm)
    report['validated'] = bool(good_pose and passed and args.confirm_order)
    if report['validated']: report['status'] = 'passed_measured_ground_checks'
    (out/'extrinsics.json').write_text(json.dumps(report, indent=2))
    if not good_pose:
        raise ValueError('Pose failed RMS/above-ground check; inspect corner order and report')
    filename = 'camera.yaml' if report['validated'] else 'camera_candidate.yaml'
    write_yaml(out/filename, k, d, size, h)
    fs = cv2.FileStorage(str(out/filename), cv2.FILE_STORAGE_APPEND)
    for name in ('T_camera_from_robot', 'T_robot_from_camera', 'T_camera_from_board', 'T_robot_from_board'):
        fs.write(name, np.array(report[name]))
    fs.write('extrinsics_validated', int(report['validated'])); fs.release()
    print(f'Saved {out}/{filename}; reprojection RMS={report["rms_px"]:.3f} px')
    print('Inspect corners_numbered.png: 0=measured origin, 0->1=board X, 0->cols=board Y.')
    if not report['validated']:
        print('CANDIDATE ONLY: confirm corner order and supply independent measured ground checks before deployment.')
    if args.check_points and not passed:
        raise ValueError('Independent ground checks failed; final camera.yaml not generated')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='mode',required=True)
    p=sub.add_parser('intrinsics',help='Automatic chessboard selection and intrinsic calibration')
    source=p.add_mutually_exclusive_group();source.add_argument('--camera',default='0');source.add_argument('--images',help='Quoted image glob')
    p.add_argument('--cols',type=int,default=9);p.add_argument('--rows',type=int,default=6)
    p.add_argument('--square-mm',type=float,required=True)
    p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480)
    p.add_argument('--views',type=int,default=25);p.add_argument('--min-views',type=int,default=15)
    p.add_argument('--timeout',type=float,default=300);p.add_argument('--min-sharpness',type=float,default=60)
    p.add_argument('--preview',action='store_true',help='Requires desktop display; default is headless')
    p.add_argument('--output',required=True,help='New output directory (must not exist)');p.set_defaults(func=intrinsics)
    p=sub.add_parser('snapshot',help='Save one original camera frame without a GUI')
    p.add_argument('--camera',default='0');p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480)
    p.add_argument('--output',required=True);p.set_defaults(func=snapshot)
    p=sub.add_parser('pick',help='Click measured ground points on an unchanged-resolution image')
    p.add_argument('--image',required=True);p.add_argument('--world',required=True);p.add_argument('--output',required=True);p.set_defaults(func=pick)
    p=sub.add_parser('ground',help='Fit ground mapping and verify independent measured points')
    p.add_argument('--intrinsics',required=True);p.add_argument('--points',required=True)
    p.add_argument('--max-error-mm',type=float,default=20);p.add_argument('--output',required=True);p.set_defaults(func=ground)
    p=sub.add_parser('extrinsics',help='Chessboard pose in robot frame and ground homography')
    p.add_argument('--intrinsics',required=True);p.add_argument('--image',required=True)
    p.add_argument('--cols',type=int,default=9);p.add_argument('--rows',type=int,default=6)
    p.add_argument('--square-mm',type=float,required=True)
    p.add_argument('--origin-x',type=float,required=True,help='Corner 0 robot x in metres (right positive)')
    p.add_argument('--origin-y',type=float,required=True,help='Corner 0 robot y in metres (forward positive)')
    p.add_argument('--yaw-deg',type=float,required=True,help='Board X angle from robot +x toward +y')
    p.add_argument('--board-height-mm',type=float,default=0,help='Printed board surface height above ground')
    p.add_argument('--flip-cols',action='store_true');p.add_argument('--flip-rows',action='store_true')
    p.add_argument('--confirm-order',action='store_true',help='User has checked numbered corner orientation')
    p.add_argument('--check-points',help='Independent ground checks JSON with image_size and check array')
    p.add_argument('--max-rms',type=float,default=.5);p.add_argument('--max-error-mm',type=float,default=20)
    p.add_argument('--output',required=True);p.set_defaults(func=extrinsics)
    args=parser.parse_args()
    if args.mode=='intrinsics' and (args.cols<3 or args.rows<3 or args.square_mm<=0 or not math.isfinite(args.square_mm) or args.min_views<10 or args.views<args.min_views or args.timeout<=0):
        parser.error('Invalid board dimensions, view counts or timeout')
    if args.mode=='ground' and (not math.isfinite(args.max_error_mm) or args.max_error_mm<=0):parser.error('Invalid error threshold')
    try:args.func(args)
    except (ValueError,cv2.error,OSError,KeyError) as e:parser.exit(1,f'ERROR: {e}\n')

if __name__=='__main__':main()
