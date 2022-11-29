import argparse
import time

try:
    import viz3d

    _with_viz3d = True
    from viz3d.window import OpenGLWindow
    from viz3d.engineprocess import *

except ...:
    _with_viz3d = False

try:
    import pyct_icp as pct

    _with_pct = True
except ...:
    _with_pct = False


def main():
    """
    Runs the script which displays the dataset in a viz3d window
    """
    if not _with_viz3d:
        print("This script requires the experimental "
              "package pyviz3d to be installed (see https://github.com/pierdell/pyviz3d)")
        raise ImportError("Could not import viz3d package")

    if not _with_pct:
        print("This script requires the package pct to be installed. "
              "Follow ct_icp's python bindings installation procedure ")
        raise ImportError("Could not import pct")
    parser = argparse.ArgumentParser(
        prog="show_nclt_dataset.py",
        description="Displays in a viz3d window a NCLT dataset",
        epilog="vi3d is a small project with old and dirty code, "
               "but is useful to display dynamic point clouds (for SLAM for instance)")
    parser.add_argument("--velodyne_hits_path", type=str, required=True)
    parser.add_argument("--sequence_name", type=str)
    args = parser.parse_args()
    seq_name = "unknown"
    if hasattr(args, "sequence_name") and args.sequence_name:
        seq_name = args.sequence_name

    window = OpenGLWindow()
    window.init()

    dataset = pct.NCLTIterator(args.velodyne_hits_path, seq_name)

    while dataset.HasNext():
        next_frame = dataset.NextFrame()
        pc = next_frame.pointcloud
        xyz = pc.GetXYZ()
        # timestamps = pc.GetTimestamps()
        window.set_pointcloud(0, xyz)
        time.sleep(0.05)

    window.close()


if __name__ == "__main__":
    main()
