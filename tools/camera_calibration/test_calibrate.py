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
            args=SimpleNamespace(intrinsics=root/'i.yaml',points=root/'p.json',output=root/'good',max_error_mm=20)
            c.ground(args);self.assertTrue((root/'good/camera.yaml').exists())
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
                max_rms=.5,max_error_mm=20,flip_cols=False,flip_rows=False,confirm_order=False,check_points=None,output=root/'candidate')
            with patch.object(c,'detect',return_value=corners):
                c.extrinsics(args)
                self.assertTrue((root/'candidate/camera_candidate.yaml').exists())
                self.assertFalse((root/'candidate/camera.yaml').exists())
                args.output=root/'validated';args.confirm_order=True;args.check_points=root/'check.json'
                c.extrinsics(args);self.assertTrue((root/'validated/camera.yaml').exists())
                args.origin_x+=.1;args.output=root/'wrong_origin'
                with self.assertRaises(ValueError):c.extrinsics(args)
                self.assertFalse((root/'wrong_origin/camera.yaml').exists())
                args.origin_x=origin[0];args.flip_cols=True;args.output=root/'wrong_order'
                with self.assertRaises(ValueError):c.extrinsics(args)
                self.assertFalse((root/'wrong_order/camera.yaml').exists())

    def test_detect_board(self):
        board=np.full((400,550),255,np.uint8)
        for y in range(7):
            for x in range(10):
                if (x+y)%2==0:board[25+y*50:25+(y+1)*50,25+x*50:25+(x+1)*50]=0
        corners=c.detect(board,(9,6));self.assertIsNotNone(corners);self.assertEqual(len(corners),54)

if __name__=='__main__':unittest.main()
