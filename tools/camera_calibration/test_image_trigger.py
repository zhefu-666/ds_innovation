import unittest
import image_trigger


class ImageTriggerTests(unittest.TestCase):
    def setUp(self):
        self.reference = image_trigger.load_reference(image_trigger.PROJECT)

    def evaluate(self, box, **changes):
        args = dict(image_size=[1280,720], pitch_ui_deg=140)
        args.update(changes)
        return image_trigger.evaluate(self.reference, box, **args)

    def test_supplied_samples(self):
        expected = {'front': ('at_front',226.0), 'correct': ('at_target',337.0),
                    'rear': ('at_rear',489.0)}
        for sample in self.reference['samples']:
            with self.subTest(role=sample['role']):
                result = self.evaluate(sample['bbox_xywh_px'])
                self.assertEqual((result['position'],result['bbox_center_px'][1]),
                                 expected[sample['role']])
                self.assertTrue(result['observation_only'])
                self.assertEqual(result['lateral_fit'],'unmeasured')

    def test_reference_boundaries_and_crossing(self):
        for center_y, position, reached, inside in [
                (225,'before_front',False,False), (226,'at_front',False,True),
                (227,'front_to_target',False,True), (336,'front_to_target',False,True),
                (337,'at_target',True,True), (338,'target_to_rear',True,True),
                (489,'at_rear',True,True), (490,'past_rear',True,False)]:
            with self.subTest(center_y=center_y):
                result=self.evaluate([541,center_y-50,193,100])
                self.assertEqual(result['position'],position)
                self.assertEqual(result['target_line_reached'],reached)
                self.assertEqual(result['within_longitudinal_reference'],inside)

    def test_new_class_samples_and_center_bounds(self):
        expected = {'injured_person': [395,236,906,479],
                    'core_supply': [268.5,236,1017,555]}
        for label, bounds in expected.items():
            for sample in self.reference['classes'][label]['samples']:
                with self.subTest(label=label, role=sample['role']):
                    result = self.evaluate(sample['bbox_xywh_px'], label=label)
                    self.assertTrue(result['within_grasp_center_range'])
                    self.assertEqual(result['center_bounds_px'], bounds)
            for cx, cy, inside in [(bounds[0],337,True),(bounds[2],337,True),
                                   (650,bounds[1],True),(650,bounds[3],True),
                                   (bounds[0]-.5,337,False),(bounds[2]+.5,337,False),
                                   (650,bounds[1]-.5,False),(650,bounds[3]+.5,False)]:
                # A large box can straddle the boundary; only its centre decides.
                result = self.evaluate([cx-100,cy-50,200,100],label=label)
                self.assertEqual(result['within_grasp_center_range'], inside)
            result = self.evaluate([600,287,100,100],label=label)
            self.assertEqual(result['position'],'at_target')
            self.assertTrue(result['target_line_reached'])

    def test_no_resolution_pitch_or_class_substitution(self):
        for change in [dict(image_size=[640,360]),dict(pitch_ui_deg=120),
                       dict(label='dangerous_object')]:
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.evaluate([541,210,213,254],**change)

    def test_invalid_box_and_unmeasured_lateral_position(self):
        for box in [[0,0,0,10],[0,0,float('nan'),10],[1200,600,200,200]]:
            with self.subTest(box=box),self.assertRaises(ValueError):
                self.evaluate(box)
        result=self.evaluate([0,210,213,254])
        self.assertEqual(result['position'],'at_target')
        self.assertEqual(result['lateral_fit'],'unmeasured')
        self.assertTrue(result['observation_only'])


if __name__=='__main__':
    unittest.main()
