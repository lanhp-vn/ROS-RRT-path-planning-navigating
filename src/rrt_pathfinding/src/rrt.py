#!/usr/bin/env python3

import random
import math
import numpy as np

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

def is_free(x, y, map_img):
    h, w = map_img.shape[:2]
    if 0 <= x < w and 0 <= y < h:
        return np.all(map_img[y, x] > 200)  # Free if pixel is bright
    return False

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def nearest(nodes, rnd_node):
    return min(nodes, key=lambda node: distance(node, rnd_node))

def steer(from_node, to_node, extend_length=10.0):
    d = distance(from_node, to_node)
    if d < extend_length:
        return Node(to_node.x, to_node.y, from_node)
    theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = int(from_node.x + extend_length * math.cos(theta))
    new_y = int(from_node.y + extend_length * math.sin(theta))
    return Node(new_x, new_y, from_node)

def check_path(from_node, to_node, map_img):
    steps = int(distance(from_node, to_node))
    for i in range(steps):
        x = int(from_node.x + (to_node.x - from_node.x) * i / steps)
        y = int(from_node.y + (to_node.y - from_node.y) * i / steps)
        if not is_free(x, y, map_img):
            return False
    return True

def rrt_planner(map_img, start_px, goal_px, max_iter=10000, goal_sample_rate=0.1, extend_length=5.0):
    start_node = Node(*start_px)
    goal_node = Node(*goal_px)
    nodes = [start_node]

    for _ in range(max_iter):
        if random.random() < goal_sample_rate:
            rnd_node = Node(goal_node.x, goal_node.y)
        else:
            rnd_node = Node(random.randint(0, map_img.shape[1] - 1),
                            random.randint(0, map_img.shape[0] - 1))
            if not is_free(rnd_node.x, rnd_node.y, map_img):
                continue

        nearest_node = nearest(nodes, rnd_node)
        new_node = steer(nearest_node, rnd_node, extend_length)

        if is_free(new_node.x, new_node.y, map_img) and check_path(nearest_node, new_node, map_img):
            nodes.append(new_node)

            if distance(new_node, goal_node) < extend_length:
                if check_path(new_node, goal_node, map_img):
                    goal_node.parent = new_node
                    nodes.append(goal_node)
                    break

    # Trace path
    path = []
    node = goal_node if goal_node in nodes else nodes[-1]
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def rrt_pathfinder(start_world, goal_world, map_img, resolution, origin):
    img_height = map_img.shape[0]
    # === World to Map pixel conversion ===
    def world_to_map(x, y):
        dx = x - origin[0]
        dy = y - origin[1]
        map_x = int(dx / resolution)
        map_y = map_img.shape[0] - int(dy / resolution)  # OpenCV Y flip
        return (map_x, map_y)

    # === Map to World conversion ===
    def map_to_world_path(path_px):
        return [
            (origin[0] + px * resolution,
             origin[1] + (img_height - py) * resolution)
            for px, py in path_px
        ]
    
    start_px = world_to_map(*start_world)
    goal_px = world_to_map(*goal_world)

    path_px = rrt_planner(map_img, start_px, goal_px)
    path_world = map_to_world_path(path_px)

    return start_px,goal_px,path_px,path_world