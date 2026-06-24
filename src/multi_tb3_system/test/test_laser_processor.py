import math
import unittest

from multi_tb3_system.perception.laser_processor import process_scan, Cluster

class TestLaserProcessor(unittest.TestCase):
    def test_process_scan_empty(self):
        # Empty ranges
        target, clusters = process_scan(
            ranges=[],
            angle_min=0.0,
            angle_increment=1.0,
            range_min=0.0,
            range_max=10.0,
            front_half_angle_deg=180.0,
            cluster_distance=0.5
        )
        self.assertEqual(len(clusters), 0)
        self.assertIsNone(target)

    def test_process_scan_single_cluster(self):
        # 4 points, 3 are valid and close together
        ranges = [float('inf'), 1.0, 1.05, 1.1, float('inf')]
        angle_min = 0.0
        angle_increment = 0.1
        range_min = 0.0
        range_max = 10.0
        cluster_tolerance = 0.2

        target, clusters = process_scan(
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_min=range_min,
            range_max=range_max,
            front_half_angle_deg=180.0,
            cluster_distance=cluster_tolerance,
            min_cluster_size=1
        )
        
        self.assertEqual(len(clusters), 1)
        self.assertEqual(len(clusters[0].points), 3)
        self.assertIsNotNone(target)
        
        # Centroid of the points x1, y1, x2, y2, x3, y3
        self.assertIsInstance(target.centroid, tuple)

if __name__ == '__main__':
    unittest.main()
