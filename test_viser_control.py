import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from viser_control import ControlState,checked_target,slew,ring_angle
class ControlTests(unittest.TestCase):
 def setUp(self):
  self.now=10.;self.s=ControlState(lambda:self.now);self.s.feedback(np.zeros((2,6)))
 def test_preview_does_not_enable_motors(self):
  self.s.target(0,[.2,0,0,0,0,0],1);self.assertEqual(self.s.enabled,[False,False])
 def test_enable_aligns_to_actual_not_preview(self):
  self.s.target(0,[.7,0,0,0,0,0],1);self.s.enable(0,1)
  np.testing.assert_array_equal(self.s.desired[0],np.zeros(6))
 def test_stale_feedback_cannot_enable(self):
  self.now+=.31
  with self.assertRaises(ValueError):self.s.enable(0,1)
 def test_one_owner_and_two_independent_arms(self):
  self.s.enable(0,1)
  with self.assertRaises(ValueError):self.s.enable(1,2)
  with self.assertRaises(ValueError):self.s.target(0,[.1,0,0,0,0,0],2)
  self.s.enable(1,1);self.s.disable(0);self.assertEqual(self.s.enabled,[False,True]);self.assertEqual(self.s.owner,1)
 def test_owner_disconnect_disables_both(self):
  self.s.enable(0,1);self.s.enable(1,1);self.s.disconnected(2);self.assertTrue(all(self.s.enabled))
  self.s.disconnected(1);self.assertEqual(self.s.enabled,[False,False]);self.assertIsNone(self.s.owner)
 def test_invalid_targets(self):
  for x in [[float('nan')]*6,[0]*7,[3,0,0,0,0,0]]:
   with self.assertRaises(ValueError):checked_target(x)
 def test_slew_is_bounded_and_no_overshoot(self):
  result=slew(np.zeros(6),np.array([1,-1,.001,0,0,0]),.15,.01)
  np.testing.assert_allclose(result,[.0015,-.0015,.001,0,0,0])
  with self.assertRaises(ValueError):slew(np.zeros(6),np.zeros(6),.15,.2)
 def test_ring_unwraps_past_pi(self):
  r=Rotation.from_rotvec([0,0,3.6]).as_matrix();self.assertAlmostEqual(ring_angle(r,3.5,0,4),3.6)
 def test_negative_axis_ring_is_joint_local(self):
  align,_=Rotation.align_vectors([[0,-1,0]],[[0,0,1]])
  global_rot=align*Rotation.from_rotvec([0,0,.7]);relative=(align.inv()*global_rot).as_matrix()
  self.assertAlmostEqual(ring_angle(relative,.6,0,4),.7)
if __name__=='__main__':unittest.main()
