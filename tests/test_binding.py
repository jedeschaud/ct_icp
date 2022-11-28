import unittest
import numpy as np

try:
    import pyct_icp as pct

    _with_pct = True
except ImportError as e:
    print(e.msg)
    _with_pct = False


class TestBinding(unittest.TestCase):

    def test_installation(self):
        self.assertEqual(True, _with_pct)  # add assertion here

    def test_pointcloud_creation(self):
        pc = pct.MakeEmptyPointCloud()
        xyz = pc.GetXYZ()
        xyz[:, :3] = 10.
        xyz_bis = pc.GetXYZ()
        print(xyz[:10])
        print(xyz_bis[:10])
        self.assertEqual(xyz_bis[0, 0], 10.)
        print(type(xyz))

    # def test_frame(self):
    #     self.test_installation()
    #
    #     frame = pct.LiDARFrame()
    #
    #     n = 100
    #     np_array = np.random.randn(n, 3)
    #     np_raw_points = np.random.randn(n, 3)
    #     timestamps = np.random.rand(n)
    #     alpha_timestamps = np.random.rand(n)
    #     index_frames = np.random.randint(0, 10, n)
    #
    #     struct_array = np.rec.fromarrays([np_raw_points, np_array, timestamps, alpha_timestamps, index_frames],
    #                                      dtype=[
    #                                          ("raw_point", 'f8', 3),
    #                                          ("pt", 'f8', 3),
    #                                          ("alpha_timestamp", 'f8', 1),
    #                                          ("timestamp", 'f8', 1),
    #                                          ("index_frame", 'i4', 1)])
    #
    #     frame.SetFrame(struct_array)
    #     array = frame.GetStructuredArrayRef()
    #
    #     diff2 = np.abs(array["pt"] - np_array).max()
    #     diff0 = np.abs(array["raw_point"] - np_raw_points).max()
    #     self.assertEqual(diff2, 0.0)
    #     self.assertEqual(diff0, 0.0)
    #
    #     array[:10]["pt"] = 0.0
    #
    #     array_bis = frame.GetStructuredArrayRef()
    #
    #     xs = array["pt"]
    #     xs_bis = array_bis["pt"]
    #     diff = np.abs(xs - xs_bis).max()
    #     self.assertEqual(diff, 0.0)
    #
    #     # Check that GetWrappingArray returns a reference
    #     # And SetFrame makes a copy
    #
    # def test_odometry(self):
    #     self.test_installation()
    #     options = pct.OdometryOptions()
    #     options.motion_compensation = pct.NONE
    #     pct.Odometry(options)


if __name__ == '__main__':
    unittest.main()
