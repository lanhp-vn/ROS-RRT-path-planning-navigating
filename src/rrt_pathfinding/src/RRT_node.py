#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray, Bool
from rrt import rrt_pathfinder

class RRTNode:
    def __init__(self):
        # Map data
        self.map_img     = None
        self.resolution  = None
        self.origin      = None
        self.width = self.height = 0

        # Publishers
        self.traj_pub = rospy.Publisher("/trajectory", Float64MultiArray, queue_size=1)
        self.ready_pub = rospy.Publisher("/path_ready", Bool, queue_size=1)

        # Subscribers
        rospy.Subscriber("/map", OccupancyGrid, self._map_callback)
        rospy.Subscriber("/start_goal", Float64MultiArray, self._start_goal_callback)

        rospy.loginfo("RRT node initialized, waiting for /map and /start_goal…")

    def _map_callback(self, msg: OccupancyGrid):
        """Convert incoming OccupancyGrid into a flipped grayscale image."""
        self.resolution = msg.info.resolution
        self.origin     = [msg.info.origin.position.x,
                           msg.info.origin.position.y]
        self.width      = msg.info.width
        self.height     = msg.info.height

        data = np.array(msg.data).reshape((self.height, self.width))
        # 255 = free, 0 = occupied, 127 = unknown
        img = np.full((self.height, self.width), 255, dtype=np.uint8)
        img[data == -1]  = 127
        img[data == 100] =   0
        # Flip Y so image coords match world coords
        self.map_img = np.flipud(img)

        rospy.loginfo(f"Map received: {self.width}×{self.height}, "
                      f"resolution={self.resolution:.3f}")

    def _start_goal_callback(self, msg: Float64MultiArray):
        """When start/goal arrives, run RRT, publish trajectory & notify readiness."""
        if self.map_img is None:
            rospy.logwarn("Map not ready; cannot plan path.")
            return

        # 1) Parse inputs
        x_s, y_s, x_g, y_g = msg.data
        start = [x_s, y_s]
        goal  = [x_g, y_g]
        rospy.loginfo(f"Planning from {start} → {goal}")

        # 2) Compute path
        start_px, goal_px, path_px, _ = rrt_pathfinder(
            start, goal, self.map_img, self.resolution, self.origin
        )
        rospy.loginfo(f"RRT found {len(path_px)} waypoints")

        # 3) Publish trajectory as flat [x0,y0, x1,y1, …]
        traj_msg = Float64MultiArray()
        traj_msg.data = [c for pt in path_px for c in pt]
        self.traj_pub.publish(traj_msg)
        rospy.loginfo("Trajectory published on /trajectory")

        # 4) Visualize & save an image
        self._visualize_and_save(path_px, start_px, goal_px)

        # 5) Notify motion_planner that path is ready
        self.ready_pub.publish(Bool(True))
        rospy.loginfo("Published path_ready = True")

    def _visualize_and_save(self, path_px, start_px, goal_px):
        """Overlay path, start and goal on the map and write out a PNG."""
        # 1) Draw on a BGR copy
        vis = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        for a, b in zip(path_px, path_px[1:]):
            cv2.line(vis, tuple(a), tuple(b), (0, 0, 255), 2)
        cv2.circle(vis, tuple(start_px), 5, (0, 255, 0), -1)
        cv2.circle(vis, tuple(goal_px),  5, (255, 0, 0), -1)
        cv2.putText(vis, 'S', tuple(start_px),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        cv2.putText(vis, 'G', tuple(goal_px),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

        # 2) Convert pixel coords back to world‐meters
        #    y needs flipping: pixel y = [0 top .. height-1 bottom]
        sx_m = self.origin[0] + start_px[0] * self.resolution
        sy_m = self.origin[1] + (self.height - start_px[1]) * self.resolution
        gx_m = self.origin[0] + goal_px[0]  * self.resolution
        gy_m = self.origin[1] + (self.height - goal_px[1])  * self.resolution

        # 3) Build a filename with meters
        fname = f"s({sx_m:.1f}_{sy_m:.1f})_g({gx_m:.1f}_{gy_m:.1f}).png"

        # 4) Save and log
        cv2.imwrite(fname, vis)
        rospy.loginfo(f"Saved trajectory image: {fname}")


if __name__ == "__main__":
    rospy.init_node("rrt_node", anonymous=True)
    node = RRTNode()
    rospy.spin()
