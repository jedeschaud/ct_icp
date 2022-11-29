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

    def test_pointcloud_basics(self):
        # ########## BASICS ########## #
        pc = pct.PointCloud()  # Create an empty point cloud
        pc.Resize(10)  # Resizes the point cloud
        self.assertEqual(pc.Size(), 10)

        xyz = pc.GetXYZ()  # The default point cloud only allocates a XYZ buffer
        self.assertEqual(type(xyz), np.ndarray)  # The GetXYZ returns a numpy array which is a view of the C++ buffer

        self.assertEqual(np.linalg.norm(xyz), 0.)  # It is null by default
        xyz[:, :3] = 10.  # Uses the standard numpy API to access / modifies the C++ buffer (No resize !)

        # xyz is a shallow copy, the buffer is modified, as shown below
        xyz_bis = pc.GetXYZ()
        self.assertEqual(xyz_bis[0, 0], 10.)  # The second xyz_bis array has the same modification than xyz

        # ########## FIELDS ########## #
        self.assertFalse(pc.HasRGBField())  # The point cloud does not yet have other fields defined
        pc.AddRGBField()  # Adds a RGB Field to the point cloud
        self.assertTrue(pc.HasRGBField())
        rgb = pc.GetRGB()
        self.assertTrue(rgb.shape == (pc.Size(), 3))

        # ########## Collection of Fields ########## #
        # pc.AddRGBField() : Throws an exception because the point cloud already has the field
        pc.AddRawPointsField()
        raw_xyz = pc.GetRawPoints()
        pc.AddWorldPointsField()
        world_xyz = pc.GetWorldPoints()
        pc.AddNormalsField()
        normals = pc.GetNormals()
        pc.AddTimestampsField()
        timestamps = pc.GetTimestamps()
        pc.AddIntensityField()
        intensity = pc.GetIntensity()

        # Fields
        for field in [raw_xyz, world_xyz, normals, timestamps, intensity]:
            self.assertEqual(field.shape[0], pc.Size())

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
