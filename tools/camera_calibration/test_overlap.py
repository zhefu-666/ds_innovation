import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import overlap

class OverlapTests(unittest.TestCase):
    def scene(self, root):
        k=np.array([[500.,0,320],[0,500.,240],[0,0,1.]])
        d=np.zeros((1,5))
        r=np.diag([1.,-1.,-1.]); t=np.array([0.,0.,1.])
        transform=np.eye(4); transform[:3,:3]=r; transform[:3,3]=t
        base=root/'base.yaml'
        fs=cv2.FileStorage(str(base),cv2.FILE_STORAGE_WRITE)
        for name,value in [('camera_matrix',k),('dist_coeffs',d),('T_camera_from_robot',transform),
                           ('image_width',640),('image_height',480),('extrinsics_validated',1),
                           ('pitch_model_reference_cdeg',-3000),('pitch_model_min_cdeg',-3000),
                           ('pitch_model_max_cdeg',-3000),('ground_pixel_domain','undistorted_pixels')]:
            fs.write(name,value)
        fs.release()
        h=np.linalg.inv(k@transform[:3,[0,1,3]])
        a=np.array([[200,200],[400,200],[400,350],[200,350],[280,270],[350,300]],dtype=float)
        shift=np.array([12.,-18.])
        holdout=np.array([[250,225],[370,260],[280,330]],dtype=float)
        data={'stationary_chassis':True,'image_size':[640,480],
            'pairs':[{'source_ui_deg':90,'target_ui_deg':120,'source_pixels':a.tolist(),'target_pixels':(a+shift).tolist()},
                     {'source_ui_deg':120,'target_ui_deg':140,'source_pixels':(a+shift).tolist(),'target_pixels':(a+2*shift).tolist()}],
            'validation':{str(angle):[{'pixel':pix.tolist(),'body_mm':ground.tolist()}
                 for pix,ground in zip(holdout+i*shift,overlap.project(h,holdout)*1000)]
                 for i,angle in enumerate((90,120,140))}}
        manifest=root/'points.json'; manifest.write_text(json.dumps(data))
        return base,manifest,data

    def test_chain_and_output_without_pnp(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); base,manifest,data=self.scene(root)
            with patch.object(cv2,'solvePnP',side_effect=AssertionError('PnP forbidden')):
                report=overlap.solve(manifest,base,root/'result',1.)
            self.assertTrue(report['passed'])
            fs=cv2.FileStorage(str(root/'result/camera-overlap.yaml'),cv2.FILE_STORAGE_READ)
            self.assertEqual(fs.getNode('overlap_ground_views').size(),3)
            self.assertEqual(fs.getNode('enable_pnp_crosscheck').real(),0)
            fs.release()

    def test_bad_independent_point_emits_no_calibration(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); base,manifest,data=self.scene(root)
            data['validation']['140'][0]['body_mm'][0]+=100
            manifest.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'Independent'):
                overlap.solve(manifest,base,root/'result',5.)
            self.assertTrue((root/'result/report.json').exists())
            self.assertFalse((root/'result/camera-overlap.yaml').exists())

    def test_no_fabricated_points_or_chassis_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); base,manifest,data=self.scene(root)
            data['stationary_chassis']=False
            manifest.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'stationary'):
                overlap.solve(manifest,base,root/'result',5.)
            with self.assertRaises(ValueError):
                overlap.pixels([],np.eye(3),np.zeros(5),(640,480))

if __name__=='__main__': unittest.main()
