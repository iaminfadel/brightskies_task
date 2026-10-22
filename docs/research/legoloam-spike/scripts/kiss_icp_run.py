import csv
import os
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP


def pcd_write(path, points):
    n = int(points.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.asarray(points, dtype=np.float32).tobytes())


bag, out = sys.argv[1], sys.argv[2]
cfg = KISSConfig()
cfg.data.max_range = 50.0
cfg.data.min_range = 0.0
cfg.data.deskew = False
cfg.mapping.voxel_size = 0.5
kiss = KissICP(cfg)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("cdr", "cdr"))
stamps, poses = [], []
n = 0
while r.has_next():
    topic, data, t = r.read_next()
    if topic != "/carla/hero/lidar":
        continue
    msg = deserialize_message(data, PointCloud2)
    pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
    pts = np.asarray(pts, dtype=np.float64)
    if pts.shape[0] < 100:
        continue
    kiss.register_frame(pts, np.zeros(pts.shape[0], dtype=np.float64))
    stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    poses.append(kiss.last_pose.copy())
    n += 1
    if n % 200 == 0:
        print("frames", n, flush=True)
poses = np.asarray(poses)
print("frames", n, "poses", poses.shape)
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
    for st, T in zip(stamps, poses):
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        w.writerow([st, x, y, z, 0.0, 0.0, 0.0, 1.0])
mp = np.asarray(kiss.local_map.point_cloud(), dtype=np.float32)
print("map points", mp.shape, flush=True)
base = os.path.splitext(out)[0]
np.save(base + "_map.npy", mp)
pcd_write(base + "_map.pcd", np.hstack([mp, np.zeros((mp.shape[0], 1), dtype=np.float32)]))
print("wrote map", base + "_map.pcd", flush=True)
