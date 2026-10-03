import importlib.util
from pathlib import Path
import unittest
import tempfile
from unittest.mock import patch
import json
from types import SimpleNamespace
import cv2
import numpy as np
spec=importlib.util.spec_from_file_location('calibrate',Path(__file__).with_name('calibrate.py'))
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)

class CalibrationTests(unittest.TestCase):
    def test_synthetic_intrinsics(self):
        k=np.array([[700.,0,320],[0,710,240],[0,0,1]])
        obj=c.object_grid((9,6),.025);images=[]
        rng=np.random.default_rng(6)
        for _ in range(25):
            r=rng.uniform(-.45,.45,3);t=np.array([-.1,-.06,rng.uniform(.5,.85)])
            pixels,_=cv2.projectPoints(obj,r,t,k,np.zeros(5));images.append(pixels)
        rms,actual,_,_=c.solve_intrinsics(images,(640,480),(9,6),.025)
        self.assertLess(rms,.001);np.testing.assert_allclose(actual,k,atol=.05)

    def test_ground_distortion_and_holdout(self):
        k=np.array([[700.,0,320],[0,710,240],[0,0,1]]);d=np.array([.12,-.04,0,0,0.])
        world=np.array([[x,y] for y in (.4,.7,1.,1.3) for x in (-.3,0,.3)],np.float32)
        # Synthetic tilted ground plane yields nontrivial lens distortion.
        obj=np.column_stack((world,np.zeros(len(world)))).astype(np.float32)
        pixels,_=cv2.projectPoints(obj,np.array([.8,0,0]),np.array([0.,-.5,1.8]),k,d)
        indices=[0,2,3,5,6,8,9,11];checks=[1,4,7,10]
        h,mask=c.fit_ground(k,d,pixels[indices],world[indices]);self.assertTrue(mask.all())
        und=cv2.undistortPoints(pixels[checks],k,d,P=k)
        actual=cv2.perspectiveTransform(und,h).reshape(-1,2)
        np.testing.assert_allclose(actual,world[checks],atol=.001)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);c.write_yaml(root/'i.yaml',k,d,(640,480))
            data={'image_size':[640,480]}
            for key,ids in [('fit',indices),('check',checks)]:
                data[key]=[dict(pixel=pixels[i,0].tolist(),world_m=world[i].tolist()) for i in ids]
            (root/'p.json').write_text(json.dumps(data))
            args=SimpleNamespace(intrinsics=root/'i.yaml',points=root/'p.json',output=root/'good',max_error_mm=20,camera_pitch_cdeg=1500)
            c.ground(args);self.assertTrue((root/'good/camera.yaml').exists())
            fs=cv2.FileStorage(str(root/'good/camera.yaml'),cv2.FILE_STORAGE_READ)
            self.assertEqual(int(fs.getNode('ground_camera_pitch_cdeg').real()),1500);fs.release()
            data['check'][0]['world_m'][0]+=.2;(root/'p.json').write_text(json.dumps(data));args.output=root/'bad'
            with self.assertRaises(ValueError):c.ground(args)
            self.assertFalse((root/'bad/camera.yaml').exists())

    def test_board_extrinsics_robot_transform_and_ground_height(self):
        k=np.array([[500.,0,320],[0,500,240],[0,0,1]])
        d=np.array([.1,-.03,0,0,0.])
        r=cv2.Rodrigues(np.array([2.4,0.,0.]))[0]
        center=np.array([.02,-.1,.8]);t=-r@center
        angle=.25;origin=(-.1,.55);height=.004
        rb=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        obj=c.object_grid((9,6),.025)
        rv=cv2.Rodrigues(r@rb)[0];tv=r@np.array([*origin,height])+t
        corners,_=cv2.projectPoints(obj,rv,tv,k,d)
        report,h=c.solve_board_pose(k,d,corners,(9,6),.025,origin,np.degrees(angle),height)
        np.testing.assert_allclose(report['camera_position_robot_m'],center,atol=1e-4)
        np.testing.assert_allclose(np.array(report['T_camera_from_robot'])@np.array(report['T_robot_from_camera']),np.eye(4),atol=1e-8)
        ground=np.array([[-.2,.4,0],[.2,.6,0],[0,1.,0]],np.float32)
        pixels,_=cv2.projectPoints(ground,cv2.Rodrigues(r)[0],t,k,d)
        und=cv2.undistortPoints(pixels,k,d,P=k)
        np.testing.assert_allclose(cv2.perspectiveTransform(und,h).reshape(-1,2),ground[:,:2],atol=1e-4)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);c.write_yaml(root/'i.yaml',k,d,(640,480));cv2.imwrite(str(root/'board.png'),np.zeros((480,640,3),np.uint8))
            data={'image_size':[640,480],'check':[dict(pixel=p.tolist(),world_m=w[:2].tolist()) for p,w in zip(pixels[:,0],ground)]}
            (root/'check.json').write_text(json.dumps(data))
            args=SimpleNamespace(intrinsics=root/'i.yaml',image=str(root/'board.png'),cols=9,rows=6,square_mm=25.,
                origin_x=origin[0],origin_y=origin[1],yaw_deg=np.degrees(angle),board_height_mm=4.,
                max_rms=.5,max_error_mm=20,camera_pitch_cdeg=2000,flip_cols=False,flip_rows=False,confirm_order=False,check_points=None,output=root/'candidate')
            with patch.object(c,'detect',return_value=corners):
                c.extrinsics(args)
                self.assertTrue((root/'candidate/camera_candidate.yaml').exists())
                self.assertFalse((root/'candidate/camera.yaml').exists())
                args.output=root/'validated';args.confirm_order=True;args.check_points=root/'check.json'
                c.extrinsics(args);self.assertTrue((root/'validated/camera.yaml').exists())
                fs=cv2.FileStorage(str(root/'validated/camera.yaml'),cv2.FILE_STORAGE_READ)
                self.assertEqual(int(fs.getNode('pitch_model_reference_cdeg').real()),2000)
                self.assertEqual(int(fs.getNode('extrinsics_validated').real()),1);fs.release()
                args.origin_x+=.1;args.output=root/'wrong_origin'
                with self.assertRaises(ValueError):c.extrinsics(args)
                self.assertFalse((root/'wrong_origin/camera.yaml').exists())
                args.origin_x=origin[0];args.flip_cols=True;args.output=root/'wrong_order'
                with self.assertRaises(ValueError):c.extrinsics(args)
                self.assertFalse((root/'wrong_order/camera.yaml').exists())

    def test_circle_board_extrinsics_writes_validated_camera(self):
        k=np.array([[500.,0,320],[0,500,240],[0,0,1]])
        d=np.array([.1,-.03,0,0,0.])
        r=cv2.Rodrigues(np.array([2.4,0.,0.]))[0]
        center=np.array([.02,-.1,.8]);t=-r@center
        angle=.25;origin=(-.1,.55);height=.004
        rb=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        obj=c.object_grid((9,6),.018)
        rv=cv2.Rodrigues(r@rb)[0];tv=r@np.array([*origin,height])+t
        centers,_=cv2.projectPoints(obj,rv,tv,k,d)
        ground=np.array([[-.2,.4,0],[.2,.6,0],[0,1.,0]],np.float32)
        pixels,_=cv2.projectPoints(ground,cv2.Rodrigues(r)[0],t,k,d)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);c.write_yaml(root/'i.yaml',k,d,(640,480))
            cv2.imwrite(str(root/'board.png'),np.zeros((480,640,3),np.uint8))
            data={'image_size':[640,480], 'check':[
                dict(pixel=p.tolist(),world_m=w[:2].tolist())
                for p,w in zip(pixels[:,0],ground)]}
            (root/'check.json').write_text(json.dumps(data))
            args=SimpleNamespace(intrinsics=root/'i.yaml',image=str(root/'board.png'),
                pattern='circles',cols=9,rows=6,square_mm=18.,
                origin_x=origin[0],origin_y=origin[1],yaw_deg=np.degrees(angle),
                board_height_mm=4.,max_rms=.5,max_error_mm=20,camera_pitch_cdeg=2000,flip_cols=False,
                flip_rows=False,confirm_order=True,check_points=root/'check.json',
                output=root/'validated')
            with patch.object(c,'detect_circle_grid',return_value=centers):
                c.extrinsics(args)
            self.assertTrue((root/'validated/camera.yaml').exists())
            report=json.loads((root/'validated/extrinsics.json').read_text())
            self.assertEqual(report['board_type'],'symmetric_circles')
            self.assertTrue(report['validated'])

    def test_far_origin_with_board_y_toward_robot(self):
        k=np.array([[500.,0,320],[0,500,240],[0,0,1]])
        d=np.zeros(5)
        robot_from_camera=np.array([-.05,-.1,.8])
        camera_from_robot=cv2.Rodrigues(np.array([2.4,0.,0.]))[0]
        robot_from_board=np.diag([1.,-1.,-1.])
        origin=np.array([-.16,.895,.015])
        obj=c.object_grid((9,6),.04)
        rv=cv2.Rodrigues(camera_from_robot@robot_from_board)[0]
        tv=camera_from_robot@(origin-robot_from_camera)
        corners,_=cv2.projectPoints(obj,rv,tv,k,d)
        report,h=c.solve_board_pose(k,d,corners,(9,6),.04,origin[:2],0,origin[2],True)
        np.testing.assert_allclose(report['camera_position_robot_m'],robot_from_camera,atol=1e-4)
        self.assertAlmostEqual(np.linalg.det(np.array(report['T_robot_from_board'])[:3,:3]),1.)
        self.assertLess(report['rms_px'],.001)
        ground=np.array([[-.2,.3,0.],[.2,.6,0.],[0.,1.,0.]],np.float32)
        pixels,_=cv2.projectPoints(ground,cv2.Rodrigues(camera_from_robot)[0],
            -camera_from_robot@robot_from_camera,k,d)
        mapped=cv2.perspectiveTransform(cv2.undistortPoints(pixels,k,d,P=k),h)
        np.testing.assert_allclose(mapped.reshape(-1,2),ground[:,:2],atol=1e-4)

    def test_pitch_range_validates_servo_model(self):
        k=np.array([[800.,0,640],[0,800,360],[0,0,1]]);d=np.zeros(5)
        def camera(deg):
            a=np.radians(deg);return np.array([[1,0,0],[0,-np.sin(a),-np.cos(a)],[0,np.cos(a),-np.sin(a)]])
        centre=np.array([0,.05,.30]);r=camera(22);T=np.eye(4);T[:3,:3]=r;T[:3,3]=-r@centre
        ground=np.array([[-.25,.5,0],[.25,.7,0],[0,1.1,0],[.1,.6,0]])
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);c.write_yaml(root/'camera.yaml',k,d,(1280,720),np.eye(3),2200)
            fs=cv2.FileStorage(str(root/'camera.yaml'),cv2.FILE_STORAGE_APPEND)
            fs.write('T_camera_from_robot',T);fs.write('extrinsics_validated',1)
            for key in ('reference','min','max'):fs.write(f'pitch_model_{key}_cdeg',2200)
            fs.release()
            def checks(deg,name,shift=0.):
                # Truth: the camera itself pitched to deg (independent of the servo model).
                rr=camera(deg);px,_=cv2.projectPoints(ground,cv2.Rodrigues(rr)[0],-rr@centre,k,d)
                data={'image_size':[1280,720],'check':[dict(pixel=p.tolist(),world_m=[w[0]+shift,w[1]]) for p,w in zip(px[:,0],ground)]}
                (root/name).write_text(json.dumps(data));return f'{int(deg*100)}:{root/name}'
            args=lambda out,*items,gap=1500:SimpleNamespace(camera=root/'camera.yaml',check=list(items),
                pivot_camera_m=[0.,0.,0.],max_error_mm=20,max_gap_cdeg=gap,output=root/out)
            c.pitch_range(args('ok',checks(12,'a.json'),checks(35,'b.json')))
            report=json.loads((root/'ok/pitch_range.json').read_text())
            self.assertTrue(report['passed']);self.assertEqual(report['valid_range_cdeg'],[1200,3500])
            self.assertLess(max(max(r['errors_mm']) for r in report['checks']),.5)
            out=cv2.FileStorage(str(root/'ok/camera.yaml'),cv2.FILE_STORAGE_READ)
            self.assertEqual(int(out.getNode('pitch_model_min_cdeg').real()),1200)
            self.assertEqual(int(out.getNode('pitch_model_max_cdeg').real()),3500);out.release()
            # A wrong measurement fails, and an untested gap is never widened over.
            with self.assertRaises(ValueError):c.pitch_range(args('bad',checks(12,'c.json',.05)))
            self.assertFalse((root/'bad/camera.yaml').exists())
            with self.assertRaises(ValueError):c.pitch_range(args('gap',checks(35,'d.json'),gap=1000))
            self.assertFalse((root/'gap/camera.yaml').exists())

    def test_detect_board(self):
        board=np.full((400,550),255,np.uint8)
        for y in range(7):
            for x in range(10):
                if (x+y)%2==0:board[25+y*50:25+(y+1)*50,25+x*50:25+(x+1)*50]=0
        corners=c.detect(board,(9,6));self.assertIsNotNone(corners);self.assertEqual(len(corners),54)

if __name__=='__main__':unittest.main()
