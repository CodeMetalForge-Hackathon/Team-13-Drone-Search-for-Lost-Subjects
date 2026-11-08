#!/usr/bin/env python3
"""Lidar-inertial odometry node (runs inside the cf_vio/openvins container next to OpenVINS).

  /imu0 (sensor_msgs/Imu) + /scan (sensor_msgs/LaserScan, RPLIDAR C1)  -->  live/lio2d.py
     --> /lio/odom        nav_msgs/Odometry, frame "lio_world" (lidar frame at the first scan), stamp = end of scan
     --> /lio/scan_world  sensor_msgs/PointCloud, the deskewed scan in lio_world (for the viewer)
     --> <out>/traj_lio.txt   TUM format: t x y z qx qy qz qw  (z = 0)
     --> <out>/lio_stats.csv  per scan: t, ok, matches, icp rms, degenerate, reanchors, ms

Usage (live/run_live.sh starts it):
  python3 lio_node.py --out /data/data/<name> [--yaw-sign 1]
"""
import argparse
import math
import os
import sys
import threading

import rospy
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lio2d import LIO2D  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="", help="folder for traj_lio.txt / lio_stats.csv ('' = don't save)")
    ap.add_argument("--yaw-sign", type=float, default=1.0,
                    help="+1 if the lidar spins about the IMU 'up' axis (C1 right side up, as on the rig), "
                         "-1 if mounted upside down")
    ap.add_argument("--min-range", type=float, default=0.35, help="drop returns closer than this (rig body)")
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("lio2d")
    lio = LIO2D(yaw_sign=args.yaw_sign, min_range=args.min_range)
    lock = threading.Lock()
    pub_odom = rospy.Publisher("/lio/odom", Odometry, queue_size=20)
    pub_pts = rospy.Publisher("/lio/scan_world", PointCloud, queue_size=5)
    ftraj = fstat = None
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        ftraj = open(os.path.join(args.out, "traj_lio.txt"), "w")
        ftraj.write("# timestamp(s) tx ty tz qx qy qz qw   (LIO, lidar frame at first scan)\n")
        fstat = open(os.path.join(args.out, "lio_stats.csv"), "w")
        fstat.write("t,ok,matches,rms_m,degenerate,reanchors,ms\n")

    def on_imu(m):
        g, a = m.angular_velocity, m.linear_acceleration
        with lock:
            lio.add_imu(m.header.stamp.to_nsec(), (g.x, g.y, g.z), (a.x, a.y, a.z))

    def on_scan(m):
        with lock:
            o = lio.add_scan(m.header.stamp.to_nsec(), m.angle_min, m.angle_increment, m.time_increment, m.ranges)
        if o is None:
            return
        t = rospy.Time(int(o["t_ns"] // 1_000_000_000), int(o["t_ns"] % 1_000_000_000))
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = t, "lio_world", "lidar0"
        od.pose.pose.position.x, od.pose.pose.position.y = o["x"], o["y"]
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(o["yaw"] / 2), math.cos(o["yaw"] / 2)
        # pose covariance[0] carries the ICP rms, [7] matches, [14] ok flag: cheap diagnostics for the viewer
        cov = [0.0] * 36
        cov[0], cov[7], cov[14], cov[21] = o["rms"] if o["rms"] == o["rms"] else -1.0, float(o["n_match"]), \
            float(o["ok"]), float(o.get("degenerate", False))
        cov[28] = o["ms"]
        od.pose.covariance = cov
        pub_odom.publish(od)
        pc = PointCloud()
        pc.header.stamp, pc.header.frame_id = t, "lio_world"
        pc.points = [Point32(float(p[0]), float(p[1]), 0.0) for p in o["pts_world"][::2]]
        pub_pts.publish(pc)
        if ftraj:
            ts = o["t_ns"] * 1e-9
            ftraj.write(f"{ts:.9f} {o['x']:.5f} {o['y']:.5f} 0 0 0 {math.sin(o['yaw'] / 2):.7f} "
                        f"{math.cos(o['yaw'] / 2):.7f}\n")
            fstat.write(f"{ts:.9f},{int(o['ok'])},{o['n_match']},{o['rms']:.5f},{int(o.get('degenerate', 0))},"
                        f"{o.get('reanchors', 0)},{o['ms']:.2f}\n")
            if lio.n_scans % 20 == 0:
                ftraj.flush()
                fstat.flush()

    rospy.Subscriber("/imu0", Imu, on_imu, queue_size=2000, tcp_nodelay=True)
    rospy.Subscriber("/scan", LaserScan, on_scan, queue_size=20)
    rospy.loginfo("lio2d: waiting for /imu0 and /scan")
    rospy.spin()
    for f in (ftraj, fstat):
        if f:
            f.close()


if __name__ == "__main__":
    main()
