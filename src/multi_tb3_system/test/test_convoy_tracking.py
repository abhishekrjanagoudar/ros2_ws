import os
import sys
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from convoy_tracking import (
    compute_goal_point,
    is_newer_breadcrumb
)

class TestConvoyTracking(unittest.TestCase):
    def test_compute_goal_point(self):
        # Create a straight line path along X axis
        path = [(float(i), 0.0) for i in range(10)] # [ (0,0), (1,0), ..., (9,0) ]
        
        # Newest breadcrumb is (9,0). If gap is 2.5, goal point should be 2.5m behind newest
        lookahead = compute_goal_point(path, 2.5)
        self.assertIsNotNone(lookahead)
        self.assertAlmostEqual(lookahead[0], 6.5)
        self.assertAlmostEqual(lookahead[1], 0.0)

        # What if gap is further than the path's total length?
        lookahead = compute_goal_point(path, 15.0)
        self.assertAlmostEqual(lookahead[0], 0.0)
        self.assertAlmostEqual(lookahead[1], 0.0)

    def test_is_newer_breadcrumb(self):
        # Signature: is_newer_breadcrumb(prev_count, prev_stamp_ns, new_count, new_stamp_ns)
        
        # New count larger
        self.assertTrue(is_newer_breadcrumb(10, 1000, 12, 1000))
        
        # Same count but newer stamp
        self.assertTrue(is_newer_breadcrumb(10, 1000, 10, 2000))
        
        # Same count, older stamp
        self.assertFalse(is_newer_breadcrumb(10, 2000, 10, 1000))
        
        # New count smaller (implies path reset, so it IS considered newer!)
        self.assertTrue(is_newer_breadcrumb(10, 1000, 2, 2000))

if __name__ == '__main__':
    unittest.main()
