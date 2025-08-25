#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray, Bool

class MotionPlanner:
    def __init__(self):
        rospy.init_node("motion_planner", anonymous=True)

        # Map parameters for pixel->world conversion
        self.resolution = None  # meters per pixel
        self.origin = None      # [x_origin, y_origin]
        self.map_height = None  # in pixels

        # Flags
        self.map_ready = False

        # Publishers
        self.start_goal_pub = rospy.Publisher(
            "/start_goal", Float64MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber("/map", OccupancyGrid, self._map_callback)
        rospy.Subscriber("/trajectory", Float64MultiArray, self._trajectory_callback)
        rospy.Subscriber("/path_ready", Bool, self._ready_callback)

        # Internal storage
        self.trajectory_px = []    # list of (px, py)
        self.trajectory_m = []     # list of (wx, wy)
        self.ready = False

        rospy.loginfo("MotionPlanner initialized: waiting for map...")

    def _map_callback(self, msg: OccupancyGrid):
        # Store map metadata from OccupancyGrid
        self.resolution = msg.info.resolution
        self.origin = [msg.info.origin.position.x,
                       msg.info.origin.position.y]
        self.map_height = msg.info.height
        self.map_ready = True
        rospy.loginfo(f"Map parameters set: resolution={self.resolution}, origin={self.origin}, height={self.map_height}")

    def _trajectory_callback(self, msg: Float64MultiArray):
        # Received pixel-based trajectory; convert to list of tuples
        data = msg.data
        self.trajectory_px = [(int(data[i]), int(data[i+1]))
                              for i in range(0, len(data), 2)]
        # Convert to world coordinates
        self.trajectory_m = [self._pixel_to_world(px, py)
                              for px, py in self.trajectory_px]
        # Print world coords trajectory
        rospy.loginfo("Trajectory in world coordinates:")
        for wx, wy in self.trajectory_m:
            print(f" -> ({wx:.2f}, {wy:.2f})")

    def _ready_callback(self, msg: Bool):
        # Indicates planning is finished
        self.ready = msg.data
        rospy.loginfo(f"Planning complete: path_ready={self.ready}")

    def _pixel_to_world(self, px, py):
        # Converts a pixel coordinate (px, py) to world (meters)
        wx = self.origin[0] + px * self.resolution
        wy = self.origin[1] + (self.map_height - py) * self.resolution
        return wx, wy

    def prompt_start_goal(self):
        # Prompt user for start and goal points
        try:
            xs = float(input("Enter start X (m): "))
            ys = float(input("Enter start Y (m): "))
            xg = float(input("Enter goal X (m): "))
            yg = float(input("Enter goal Y (m): "))
            return xs, ys, xg, yg
        except ValueError:
            rospy.logerr("Invalid input: please enter numeric values.")
            return self.prompt_start_goal()

    def publish_start_goal(self, xs, ys, xg, yg):
        msg = Float64MultiArray()
        msg.data = [xs, ys, xg, yg]
        self.start_goal_pub.publish(msg)
        rospy.loginfo(f"Sent start=({xs}, {ys}), goal=({xg}, {yg}) to /start_goal")

    def run(self):
        # Main loop: wait for map, then cycle prompt->plan->repeat
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if not self.map_ready:
                rospy.logwarn_once("Waiting for map before planning...")
                rate.sleep()
                continue

            # Prompt and publish start/goal
            xs, ys, xg, yg = self.prompt_start_goal()
            self.ready = False
            self.publish_start_goal(xs, ys, xg, yg)

            # Wait until RRT_node signals path_ready
            while not self.ready and not rospy.is_shutdown():
                rospy.sleep(0.1)

            # At this point, trajectory has been printed
            rospy.loginfo("Ready for next start/goal pair.")

        rospy.loginfo("MotionPlanner shutting down.")


if __name__ == "__main__":
    planner = MotionPlanner()
    planner.run()
