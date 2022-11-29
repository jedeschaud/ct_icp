import unittest
import numpy as np
from pathlib import Path

try:
    import pyct_icp as pct
    import os

    _with_pct = True
except ImportError as e:
    print(e.msg)
    _with_pct = False


class TestBinding(unittest.TestCase):

    def test_installation(self):
        self.assertEqual(True, _with_pct)  # add assertion here

    def test_basic_types(self):
        # ####### SE3 ####### #
        transform = pct.SE3()
        rot = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        transform.SetRotation(rot)
        rot_bis = transform.Rotation()
        self.assertLess(np.linalg.norm(rot - rot_bis), 1.e-10)

        # The binding covers the operator * to transform poses
        t1 = pct.SE3.Random()
        t2 = pct.SE3.Random(10., 3.14)
        t3 = t1 * t2
        t4 = t1.Inverse() * t2

        t3_mat = t1.Matrix().dot(t2.Matrix())
        diff_t3 = np.linalg.norm(t3_mat - t3.Matrix())
        diff_t4 = np.linalg.norm(np.linalg.inv(t1.Matrix()).dot(t2.Matrix()) - t4.Matrix())

        self.assertLess(diff_t3, 1.e-10)
        self.assertLess(diff_t4, 1.e-10)

        # The binding also covers applying a transform to a point
        xyz = np.array([1, 2, 3])
        result = t1 * xyz
        mat = t1.Matrix()
        rot = mat[:3, :3]
        tr = mat[:3, 3]
        result_bis = rot.dot(xyz) + tr

        t1 * np.random.randn(3, 1)
        t1 * np.random.randn(3, )
        # Invalid t1 * np.random.randn(1, 3)

        diff_result = np.linalg.norm(result_bis - result)
        self.assertLess(diff_result, 1.e-10)

        # Angular distance between two poses
        self.assertGreater(pct.AngularDistance(t1, t2), 0.)
        self.assertEqual(pct.AngularDistance(t1, t1), 0.)

        # ####### Pose ####### #
        pose1 = pct.Pose()
        pose1.pose = pct.SE3.Random()
        pose1.dest_timestamp = 0.
        pose1.dest_frame_id = 0
        pose2 = pct.Pose()
        pose2.pose = pct.SE3.Random()
        pose2.dest_timestamp = 1.
        pose2.dest_frame_id = 1

        pose_mid = pose1.InterpolatePose(pose2, 0.5, 0)
        xyz_mid = pose_mid.pose.tr
        xyz_mid_real = 0.5 * (pose1.pose.tr + pose2.pose.tr)
        diff = np.linalg.norm(xyz_mid - xyz_mid_real)
        self.assertLess(diff, 1.e-10)

        # Apply transformation on the point
        xyz = np.random.randn(3, )
        xyz_mid = pose_mid * xyz
        xyz_mid2 = pose1.ContinuousTransform(xyz, pose2, 0.5)
        diff = np.linalg.norm(xyz_mid - xyz_mid2)
        self.assertLess(diff, 1.e-10)
        print(pose1.pose)

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

    def test_io(self):
        # ########### READ / WRITE Point Cloud  ########### #
        pc = pct.PointCloud()
        pc.Resize(129)
        xyz = pc.GetXYZ()
        pc.AddRGBField()
        rgb = pc.GetRGB()

        xyz[:, :] = np.random.randn(129, 3)
        rgb[:, :] = np.random.randn(129, 3)

        pct.WritePointCloudAsPLY("/tmp/test.ply", pc)
        pc_bis = pct.ReadPointCloudFromPLY("/tmp/test.ply")
        self.assertTrue(pc_bis.HasRGBField())
        self.assertEqual(pc_bis.Size(), pc.Size())

        xyz_bis = pc_bis.GetXYZ()
        rgb_bis = pc_bis.GetRGB()

        diff_xyz = np.linalg.norm(xyz_bis - xyz)
        diff_rgb = np.linalg.norm(rgb_bis - rgb)
        self.assertLess(diff_xyz, 1.e-10)
        self.assertLess(diff_rgb, 1.e-10)

        # ########### Poses  ########### #
        def rand_pose():
            _pose = pct.Pose()
            _pose.pose = pct.SE3.Random()
            return _pose

        poses = [rand_pose() for _ in range(10)]
        pct.WritePosesAsPLY("/tmp/test_poses.ply", poses)
        poses_bis = pct.ReadPosesFromPLY("/tmp/test_poses.ply")

        self.assertEqual(len(poses), len(poses_bis))
        self.assertEqual(len(poses), 10)
        for pose1, pose2 in zip(poses, poses_bis):
            diff = np.linalg.norm(pose1.Matrix() - pose2.Matrix())
            self.assertLess(diff, 1.e-10)

    def test_ct_icp_types(self):
        # ####### LidarIMUFrame ####### #
        frame = pct.LidarIMUFrame()
        frame.pointcloud = pct.PointCloud()
        frame.pointcloud.Resize(100)
        frame.imu_data = [pct.ImuData() for _ in range(20)]

        # #######   ####### #

    def test_ct_icp_datasets_NCLT(self):
        env_var = os.environ
        if "NCLT_HITS_FILE" not in env_var:
            print("Cannot test the NCLT dataset, the var NCLT_HITS_FILE not present in the environment")
            return
        hits_path = env_var["NCLT_HITS_FILE"]
        if not Path(hits_path).exists():
            print("Cannot test the NCLT dataset, the var NCLT_HITS_FILE points to an non-existing file on disk")
            return
        dataset = pct.NCLTIterator(hits_path, "2012-01-08")
        self.assertTrue(dataset.HasNext())
        next = dataset.NextFrame()
        next2 = dataset.NextFrame()
        self.assertTrue(next.pointcloud.HasTimestampsField())
        self.assertTrue(next2.pointcloud.HasTimestampsField())
        self.assertTrue(next.pointcloud.Size() > 0)

    def test_ct_icp_datasets_KITTI_RAW(self):
        env_var = os.environ
        if "KITTI_PATH" not in env_var:
            print("Cannot Test the KITTI_RAW dataset, the var KITTI_PATH not present in the environment")
            return

        # ########## PLY DIRECTORY ########## #
        root_path = str(Path(env_var["KITTI_PATH"]) / "01" / "frames")
        if not Path(root_path).exists():
            print("Cannot Test the KITTI dataset. The var KITTI_PATH points to a non existing path on disk")
            return
        sequence = pct.PLYDirectory(root_path)
        paths = sequence.GetFilePaths()
        self.assertTrue(sequence.HasNext())
        self.assertGreater(len(paths), 0)
        self.assertEqual(len(paths), sequence.NumFrames())
        next = sequence.NextFrame()
        next2 = sequence.NextFrame()
        self.assertTrue(next.pointcloud.HasTimestampsField())
        self.assertTrue(next2.pointcloud.HasTimestampsField())

        xyz = next.pointcloud.GetXYZ()
        timestamps = next.pointcloud.GetTimestamps()

        self.assertGreater(xyz.shape[0], 1000)

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
