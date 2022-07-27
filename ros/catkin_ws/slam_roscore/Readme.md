# ROSCore nodes

> The ROSCore package defines the following set of utility ROS nodes:
>
> - **print_pointcloud_schema**: A RosNode which prints the pointcloud schema for a given topic (see usage below)

# Usages

> Below the usage of the different utility ROS nodes are presented

## print_pointcloud_schema

## rosbag_to_ply

> A Node which saves point cloud messages as PLY files in a directory
>
> `./rosbag_to_ply _directory_path:=<path-to-the-output-directory>` (in a console)
>
> `rosbag play <remap-the-pointcloud-topic-to /pointcloud> -r 0.3` (play the rosbag file to save to disk)
