import time
import unittest
import numpy as np
from pathlib import Path

print("[DEBUG] REMOVE viz3d dependency")
from viz3d.window import OpenGLWindow
from viz3d.engineprocess import *

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

    def test_ct_icp_map(self):
        # ########## MAP ########## #
        pc = pct.PointCloud()
        N = 10000
        pc.Resize(N)
        xyz = pc.GetXYZ()
        xyz[:] = np.random.rand(N, 3) * 30.  # Random points at a scale of 60m
        xyz[:, 2] = 0.

        default_map_options = pct.Map_Options()
        resolution_param = pct.Map_ResolutionParam()
        resolution_param.resolution = 4.
        resolution_param.min_distance_between_points = 1.
        resolution_param.max_num_points = 20

        default_map_options.resolutions = [resolution_param]
        map = pct.MultipleResolutionVoxelMap(default_map_options)
        map.InsertPointCloud(pc, [pct.Pose()])
        map_points = map.GetMapPoints(0)
        map_points_bis = map.MapAsPointCloud()
        print(map_points.GetXYZ())

    def test_ct_icp_cticp(self):
        # ########### CTICPOptions ########### #
        # Parse / Save to YAML
        yaml_options = r"""
        num_iters_icp: 10
        parametrization: SIMPLE 
        distance: POINT_TO_POINT
        solver: CERES
        max_num_residuals: 700 
        min_num_residuals: 200
        weighting_scheme: ALL
        weight_alpha: 1. 
        weight_neighborhood: 0.2
        power_planarity: 2.0
        max_number_neighbors: 40
        min_number_neighbors: 10
        threshold_voxel_occupancy: 1
        """

        options = pct.CTICPOptionsFromYAMLStr(yaml_options)
        self.assertEqual(options.num_iters_icp, 10)
        self.assertEqual(options.parametrization, pct.POSE_PARAMETRIZATION.SIMPLE)
        self.assertEqual(options.distance, pct.ICP_DISTANCE.POINT_TO_POINT)
        self.assertEqual(options.solver, pct.CT_ICP_SOLVER.CERES)
        self.assertEqual(options.max_num_residuals, 700)
        self.assertEqual(options.min_num_residuals, 200)
        self.assertEqual(options.weighting_scheme, pct.WEIGHTING_SCHEME.ALL)
        self.assertEqual(options.weight_alpha, 1.)
        self.assertEqual(options.weight_neighborhood, 0.2)
        self.assertEqual(options.power_planarity, 2.)
        self.assertEqual(options.max_number_neighbors, 40)
        self.assertEqual(options.min_number_neighbors, 10)
        self.assertEqual(options.threshold_voxel_occupancy, 1)

        # ########### CTICP Registration ########### #
        # Build the Map         
        map_points = pct.PointCloud()
        N_map = 10000
        map_points.Resize(N_map)
        xyz = map_points.GetXYZ()
        xyz[:, :2] = np.random.rand(N_map, 2) * 1. # Z = 0 Plane

        # Build the Registration point cloud
        N_cloud = 100
        cloud = pct.PointCloud()
        cloud.Resize(N_cloud)

        # Add The required point cloud field (The ICP needs Timestamps, RawPoints and WorldPoints field)
        cloud.SetRawPointsFromXYZ() # Sets the Raw Points from the XYZ field
        cloud.AddTimestampsField()  # Adds an empty Timestamps Field
        cloud.AddWorldPointsField() # Adds an empty World Points Field

        xyz = cloud.GetXYZ()
        xyz[:, :2] = np.random.rand(N_cloud, 2)
        xyz[:, 2] += 0.1 # Z = 0.5 plane

        world_points = cloud.GetWorldPoints()
        world_points[:] = np.array(xyz)
        timestamps = cloud.GetTimestamps()
        timestamps[:] = 1.

        # Build the map
        default_map_options = pct.Map_Options()
        resolution_param = pct.Map_ResolutionParam()
        resolution_param.resolution = 0.3 # 10 cm per voxel
        resolution_param.min_distance_between_points = 0.01 #
        resolution_param.max_num_points = 100

        default_map_options.resolutions = [resolution_param]
        map = pct.MultipleResolutionVoxelMap(default_map_options)
        map.InsertPointCloud(map_points, [pct.Pose()])

        # Build the Initial Trajectory
        pose_to_update = pct.TrajectoryFrame()
        pose_to_update.begin_pose.dest_timestamp = 0.
        pose_to_update.end_pose.dest_timestamp = 1.
        pose_to_update.begin_pose.dest_frame_id = 0
        pose_to_update.end_pose.dest_frame_id = 1 

        yaml_options = r"""
        num_iters_icp: 8 
        parametrization: SIMPLE 
        distance: POINT_TO_PLANE
        solver: CERES
        # max_num_residuals: 700 
        weighting_scheme: ALL
        weight_alpha: 1. 
        weight_neighborhood: 0.2
        power_planarity: 2.0
        max_number_neighbors: 40
        min_number_neighbors: 10
        threshold_voxel_occupancy: 1
        """
        options = pct.CTICPOptionsFromYAMLStr(yaml_options)
        icp = pct.CTICP_Registration(options)
        result = icp.Register(map, cloud, pose_to_update)

        # Verify that ICP correctly did register the frame onto the plane
        self.assertLess(abs(pose_to_update.end_pose.pose.tr[2] + 0.1), 1.e-8)


        


        





if __name__ == '__main__':
    unittest.main()
