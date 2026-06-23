import os
import sys
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from motion_controller import PursuitController, slew
from follower_state import FollowerState
from safety_controller import SafetyController

class TestMotionController(unittest.TestCase):
    def test_slew(self):
        # test positive step
        self.assertAlmostEqual(slew(0.0, 1.0, 0.5), 0.5)
        # test negative step
        self.assertAlmostEqual(slew(1.0, 0.0, 0.5), 0.5)
        # test target within step
        self.assertAlmostEqual(slew(0.0, 0.2, 0.5), 0.2)
        # test exactly at target
        self.assertAlmostEqual(slew(1.0, 1.0, 0.5), 1.0)

    def test_motion_controller_initialization(self):
        safety = SafetyController(safe_distance=0.2)
        controller = PursuitController(
            convoy_slot=2,
            convoy_spacing=0.5,
            lookahead_distance=0.4,
            kp_linear=1.0,
            kp_angular=2.0,
            goal_tol=0.4,
            costmap_size=1.0,
            costmap_resolution=0.1,
            costmap_stale_timeout=1.0,
            detour_forward_min_vel=0.05,
            stationary_deadlock_timeout=5.0,
            search_angular_velocity=0.3,
            breadcrumb_timeout=2.0,
            max_lin=0.22,
            max_ang=1.0,
            safety=safety
        )
        
        # We start in HOLD because PursuitController doesn't have a state initialized
        # to SEARCHING. Wait, the state is stateless, returned by `step`.
        
        # In search mode, without path, step should return zero velocities and SEARCHING
        v, w, is_emerg = controller.step(None, None, None, 1000)
        self.assertEqual(v, 0.0)
        self.assertEqual(w, 0.3)  # Search angular velocity
        self.assertFalse(is_emerg)

if __name__ == '__main__':
    unittest.main()
