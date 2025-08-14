import rclpy, math, numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from .inverse_sensor_model import integrate_scan_logodds, logodds_to_occ
from .grid_utils import world_to_grid, grid_to_world
from .a_star_planner import a_star

class GridSlamNode(Node):
    def __init__(self):
        super().__init__('grid_slam')
        self.declare_parameters('', [
            ('frame_ids.map', 'map'), ('frame_ids.odom', 'odom'), ('frame_ids.base', 'base_footprint'),
            ('grid.resolution', 0.05), ('grid.size_x', 400), ('grid.size_y', 400),
            ('grid.origin_x', -10.0), ('grid.origin_y', -10.0),
            ('sensor.max_range', 3.5), ('sensor.hit_logit', 2.0), ('sensor.miss_logit', -1.0), ('sensor.free_decay', 0.95),
            ('publish.map_rate_hz', 2.0),
            ('planner.goal_x', 2.0), ('planner.goal_y', 0.0), ('planner.inflate_radius_cells', 2)
        ])
        p = self.get_parameters_by_prefix('')
        self.frame_map = p['frame_ids.map'].value; self.frame_odom = p['frame_ids.odom'].value; self.frame_base = p['frame_ids.base'].value
        self.res = p['grid.resolution'].value; self.nx = p['grid.size_x'].value; self.ny = p['grid.size_y'].value
        self.ox = p['grid.origin_x'].value; self.oy = p['grid.origin_y'].value
        self.max_range = p['sensor.max_range'].value; self.hit_logit = p['sensor.hit_logit'].value
        self.miss_logit = p['sensor.miss_logit'].value; self.decay = p['sensor.free_decay'].value
        self.map_pub_rate = p['publish.map_rate_hz'].value
        self.goal_xy = (p['planner.goal_x'].value, p['planner.goal_y'].value)
        self.inflate_r = p['planner.inflate_radius_cells'].value

        self.logodds = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.br = TransformBroadcaster(self)
        self.curr_pose = (0.0,0.0,0.0)
        self.timer = self.create_timer(1.0/self.map_pub_rate, self.publish_map)

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        th = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.curr_pose = (x,y,th)
        # publish map->base transform for RViz
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_map
        t.child_frame_id = self.frame_base
        t.transform.translation.x = x; t.transform.translation.y = y
        t.transform.rotation = q
        self.br.sendTransform(t)

    def scan_cb(self, msg: LaserScan):
        x,y,th = self.curr_pose
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        integrate_scan_logodds(self.logodds, self.ox, self.oy, self.res, x, y, th, angles, ranges,
                               self.max_range, self.hit_logit, self.miss_logit, self.decay)
        # Simple local plan to a fixed goal (for demo)
        gx, gy = self.goal_xy
        sgx, sgy = world_to_grid(x, y, self.ox, self.oy, self.res)
        ggx, ggy = world_to_grid(gx, gy, self.ox, self.oy, self.res)
        occ = logodds_to_occ(self.logodds)
        path_cells = a_star(occ, (sgx, sgy), (ggx, ggy), inflate=self.inflate_r)
        if path_cells:
            path = Path()
            path.header.frame_id = self.frame_map
            for (cx, cy) in path_cells:
                wx, wy = grid_to_world(cx, cy, self.ox, self.oy, self.res)
                ps = PoseStamped()
                ps.header.frame_id = self.frame_map
                ps.pose.position.x = wx; ps.pose.position.y = wy
                path.poses.append(ps)
            self.path_pub.publish(path)
            # Follow first segment roughly
            if len(path_cells) > 2:
                nx, ny = path_cells[min(5, len(path_cells)-1)]
                wx, wy = grid_to_world(nx, ny, self.ox, self.oy, self.res)
                self.go_to(wx, wy, x, y, th)

    def go_to(self, tx, ty, x, y, th):
        dx, dy = tx-x, ty-y
        ang = math.atan2(dy, dx)
        ang_err = math.atan2(math.sin(ang-th), math.cos(ang-th))
        tw = Twist()
        if abs(ang_err) > 0.2:
            tw.angular.z = 0.7 if ang_err>0 else -0.7
        else:
            tw.linear.x = 0.25
        self.cmd_pub.publish(tw)

    def publish_map(self):
        occ = logodds_to_occ(self.logodds)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_map
        msg.info.resolution = self.res
        msg.info.width = self.nx; msg.info.height = self.ny
        msg.info.origin.position.x = self.ox; msg.info.origin.position.y = self.oy
        msg.data = occ.flatten().tolist()
        self.map_pub.publish(msg)

def main():
    rclpy.init()
    node = GridSlamNode()
    rclpy.spin(node)
    rclpy.shutdown()
