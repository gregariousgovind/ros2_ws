import rclpy, math, numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from .a_star_core import a_star_costmap, inflate_obstacles

class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        self.declare_parameters('', [
            ('frame_ids.map', 'map'), ('frame_ids.base', 'base_footprint'),
            ('planner.inflate_radius_cells', 2), ('planner.max_linear', 0.3),
            ('planner.max_angular', 1.0), ('planner.goal_tolerance_m', 0.15),
            ('topics.map', '/map'), ('topics.traversability_map', '/traversability_map'),
            ('topics.goal', '/goal_pose'), ('topics.path', '/path'), ('topics.cmd_vel', '/cmd_vel')
        ])
        p = self.get_parameters_by_prefix('')
        self.frame_map = p['frame_ids.map'].value; self.frame_base = p['frame_ids.base'].value
        self.inflate = p['planner.inflate_radius_cells'].value
        self.vmax = p['planner.max_linear'].value; self.wmax = p['planner.max_angular'].value
        self.goal_tol = p['planner.goal_tolerance_m'].value
        self.topic_map = p['topics.map'].value
        self.topic_trav = p['topics.traversability_map'].value
        self.topic_goal = p['topics.goal'].value
        self.topic_path = p['topics.path'].value
        self.topic_cmd = p['topics.cmd_vel'].value

        self.map = None
        self.trav = None
        self.goal = None
        self.odom_xyth = (0.0,0.0,0.0)
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self)

        self.pub_path = self.create_publisher(Path, self.topic_path, 10)
        self.pub_cmd  = self.create_publisher(Twist, self.topic_cmd, 10)
        self.create_subscription(OccupancyGrid, self.topic_map, self.map_cb, 10)
        self.create_subscription(OccupancyGrid, self.topic_trav, self.trav_cb, 10)
        self.create_subscription(PoseStamped, self.topic_goal, self.goal_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 50)

        self.timer = self.create_timer(0.2, self.tick)

    def map_cb(self, msg): self.map = msg
    def trav_cb(self, msg): self.trav = msg  # same metadata as map; values are 0..100 costs
    def goal_cb(self, msg): self.goal = msg
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x; y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        th = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.odom_xyth = (x,y,th)

    def tick(self):
        if self.map is None or self.goal is None:
            return
        # choose cost source: DL traversability if available, otherwise occupancy
        H, W = self.map.info.height, self.map.info.width
        res = self.map.info.resolution
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y

        if self.trav is not None:
            cost = np.array(self.trav.data, dtype=np.int16).reshape(H,W)
        else:
            occ = np.array(self.map.data, dtype=np.int16).reshape(H,W)
            occ = inflate_obstacles(occ, self.inflate)
            cost = occ  # 0..100, thresholded inside planner

        x,y,th = self.odom_xyth
        sx = int((x - ox)/res); sy = int((y - oy)/res)
        gx = int((self.goal.pose.position.x - ox)/res)
        gy = int((self.goal.pose.position.y - oy)/res)
        if not (0 <= sx < W and 0 <= sy < H and 0 <= gx < W and 0 <= gy < H):
            return

        path_cells = a_star_costmap(cost, (sx,sy), (gx,gy))
        if not path_cells:
            self.get_logger().warn('A* failed to find a path')
            return

        # publish Path for RViz
        path = Path(); path.header.frame_id = self.frame_map
        for cx,cy in path_cells:
            px = cx*res + ox + 0.5*res; py = cy*res + oy + 0.5*res
            ps = PoseStamped(); ps.header.frame_id = self.frame_map
            ps.pose.position.x = px; ps.pose.position.y = py
            path.poses.append(ps)
        self.pub_path.publish(path)

        # simple follower: look-ahead index
        look = min(5, len(path_cells)-1)
        tx = path.poses[look].pose.position.x; ty = path.poses[look].pose.position.y
        self.follow(tx, ty, x, y, th)

        # stop when within tolerance
        dx = self.goal.pose.position.x - x; dy = self.goal.pose.position.y - y
        if math.hypot(dx,dy) < self.goal_tol:
            self.pub_cmd.publish(Twist())

    def follow(self, tx, ty, x, y, th):
        ang = math.atan2(ty - y, tx - x)
        ang_err = math.atan2(math.sin(ang-th), math.cos(ang-th))
        tw = Twist()
        if abs(ang_err) > 0.2:
            tw.angular.z = max(-self.wmax, min(self.wmax, 2.0*ang_err))
        else:
            tw.linear.x = self.vmax
        self.pub_cmd.publish(tw)

def main():
    rclpy.init()
    rclpy.spin(AStarNode())
    rclpy.shutdown()
