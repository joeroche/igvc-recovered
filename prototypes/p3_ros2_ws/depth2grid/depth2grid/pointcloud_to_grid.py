#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.parameter import Parameter

def bresenham(x0, y0, x1, y1):
    """Bresenham's line algorithm: yields integer points on the line"""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dy <= dx:
        err = dx // 2
        while x != x1:
            yield x, y
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
        yield x1, y1
    else:
        err = dy // 2
        while y != y1:
            yield x, y
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
        yield x1, y1

class PointCloudToGrid(Node):
    def __init__(self):
        super().__init__('pointcloud_to_grid')

        # parameters
        self.declare_parameter('grid_resolution', 0.05)   # meters per cell
        self.declare_parameter('grid_width', 200)        # cells
        self.declare_parameter( 'grid_height', 200)       # cells
        self.declare_parameter('grid_origin_x', -5.0)    # meters (world coords of cell 0)
        self.declare_parameter('grid_origin_y', -5.0)
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 5.0)
        self.declare_parameter('points_topic', '/camera/depth/points')

        self.resolution = self.get_parameter('grid_resolution').value
        self.width = self.get_parameter('grid_width').value
        self.height = self.get_parameter('grid_height').value
        self.origin_x = self.get_parameter('grid_origin_x').value
        self.origin_y = self.get_parameter('grid_origin_y').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        points_topic = self.get_parameter('points_topic').value

        # occupancy grid: -1 unknown, 0 free, 100 occupied
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)

        self.pub = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)
        self.sub = self.create_subscription(PointCloud2, points_topic, self.pc_callback, 10)

        self.timer = self.create_timer(0.5, self.publish_grid)  # publish at 2 Hz

        self.get_logger().info(f"PointCloud->Grid listening to: {points_topic}")

    def world_to_cell(self, x, y):
        cx = int(math.floor((x - self.origin_x) / self.resolution))
        cy = int(math.floor((y - self.origin_y) / self.resolution))
        return cx, cy

    def inside(self, cx, cy):
        return 0 <= cx < self.width and 0 <= cy < self.height

    def reset_grid(self):
        self.grid.fill(-1)

    def pc_callback(self, msg):
        # Clear grid for each frame (simple approach). For persistent mapping, remove this reset and implement updating logic.
        self.reset_grid()

        # read points in native frame of the pointcloud (assumes pointcloud already in your mapping frame)
        for p in pc2.read_points(msg, skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            if not (self.z_min <= z <= self.z_max):
                continue
            # assume camera frame: x forward, y left, z up; adjust as needed
            cx, cy = self.world_to_cell(x, y)
            if not self.inside(cx, cy):
                continue

            # mark free along the ray from origin cell to point cell
            # origin in world for raycasting: choose (0,0) in this example -> convert to cell
            origin_cell_x, origin_cell_y = self.world_to_cell(0.0, 0.0)
            for sx, sy in bresenham(origin_cell_x, origin_cell_y, cx, cy):
                if not self.inside(sx, sy):
                    break
                # mark free (0) along the ray except endpoint
                if sx == cx and sy == cy:
                    self.grid[sy, sx] = 100  # occupied
                else:
                    # only set free if unknown
                    if self.grid[sy, sx] == -1:
                        self.grid[sy, sx] = 0

    def publish_grid(self):
        og = OccupancyGrid()
        og.header = Header()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'camera_link'  # change if needed
        meta = MapMetaData()
        meta.resolution = self.resolution
        meta.width = int(self.width)
        meta.height = int(self.height)
        meta.origin.position.x = float(self.origin_x)
        meta.origin.position.y = float(self.origin_y)
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0
        og.info = meta
        # row-major: OccupancyGrid.data is a flattened array in row-major order, starting with (0,0)
        og.data = [int(x) for x in self.grid.flatten(order='C')]
        self.pub.publish(og)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
