import argparse
import csv
import os
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


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


class Saver(Node):
    def __init__(self, outdir):
        super().__init__("wayfinder_map_saver")
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.cloud_chunks = []
        self.traj = []
        self.counts = {"input": [], "projected": [], "segmented": [], "ground": []}
        self.done = False
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(PointCloud2, "/laser_cloud_surround", self.on_map, qos)
        self.create_subscription(PointCloud2, "/carla/hero/lidar", lambda m: self.on_count("input", m), qos)
        self.create_subscription(PointCloud2, "/full_cloud_projected", lambda m: self.on_count("projected", m), qos)
        self.create_subscription(PointCloud2, "/segmented_cloud", lambda m: self.on_count("segmented", m), qos)
        self.create_subscription(PointCloud2, "/ground_cloud", lambda m: self.on_count("ground", m), qos)
        self.create_subscription(Odometry, "/aft_mapped_to_init", self.on_odom, 10)
        self.create_timer(5.0, self.report)
        self.start = time.time()
        self.get_logger().info(f"saving to {outdir}")

    def stamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_map(self, msg):
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
            )
        except Exception:
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            if pts.size:
                pts = np.hstack([pts, np.zeros((pts.shape[0], 1), dtype=pts.dtype)])
        if pts.size:
            self.cloud_chunks.append(np.asarray(pts, dtype=np.float32))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.traj.append([self.stamp(msg), p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def on_count(self, name, msg):
        self.counts[name].append([self.stamp(msg), msg.width])

    def report(self):
        n = sum(c.shape[0] for c in self.cloud_chunks)
        self.get_logger().info(
            f"map messages={len(self.cloud_chunks)} points={n} traj={len(self.traj)} "
            f"input={len(self.counts['input'])} projected={len(self.counts['projected'])}"
        )

    def save(self):
        if self.done:
            return
        self.done = True
        path = os.path.join(self.outdir, "map_raw.pcd")
        if self.cloud_chunks:
            pts = np.vstack(self.cloud_chunks)
            np.save(os.path.join(self.outdir, "map_raw.npy"), pts)
            pcd_write(path, pts)
            self.get_logger().info(f"wrote {path} with {pts.shape[0]} points")
            # voxel downsample at 0.1 m
            keys = np.floor(pts[:, :3].astype(np.float64) / 0.1).astype(np.int64)
            _, idx = np.unique(keys, axis=0, return_index=True)
            ds = pts[np.sort(idx)]
            pcd_write(os.path.join(self.outdir, "map_0p1.pcd"), ds)
            self.get_logger().info(f"wrote map_0p1.pcd with {ds.shape[0]} points")
        with open(os.path.join(self.outdir, "trajectory.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
            w.writerows(self.traj)
        with open(os.path.join(self.outdir, "point_counts.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["topic", "stamp", "width"])
            for name, rows in self.counts.items():
                for r in rows:
                    w.writerow([name] + r)
        self.get_logger().info("saved trajectory and counts")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Saver(args.outdir)
    def stop(*_):
        node.get_logger().info("stopping")
        node.save()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.save()
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()
