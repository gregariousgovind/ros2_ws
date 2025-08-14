import rclpy, math, numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from .grid_utils import world_to_grid, grid_to_world
from .motion_models import odom_to_delta, motion_kernel
from .sensor_models import expected_ranges_from_map, likelihood_beam_model

class MarkovLocalizationNode(Node):
    def __init__(self):
        super().__init__('markov_localization')
        self.declare_parameters('', [
            ('frame_ids.map', 'map'), ('frame_ids.odom', 'odom'), ('frame_ids.base', 'base_footprint'),
            ('grid.resolution', 0.05), ('grid.size_x', 200), ('grid.size_y', 200), ('grid.theta_bins', 36),
            ('grid.origin_x', -5.0), ('grid.origin_y', -5.0),
            ('motion_model.trans_sigma', 0.05), ('motion_model.rot_sigma', 0.05), ('motion_model.trans_rot_sigma', 0.02),
            ('sensor_model.z_hit', 0.9), ('sensor_model.z_rand', 0.1), ('sensor_model.sigma_hit', 0.2),
            ('sensor_model.max_range', 3.5),
            ('update.skip_beams', 3), ('update.downsample_theta', 2), ('publish_rate_hz', 8.0)
        ])
        p = self.get_parameters_by_prefix('')
        self.frame_map = p['frame_ids.map'].value; self.frame_odom = p['frame_ids.odom'].value; self.frame_base = p['frame_ids.base'].value
        self.res = p['grid.resolution'].value; self.nx = p['grid.size_x'].value; self.ny = p['grid.size_y'].value
        self.nth = p['grid.theta_bins'].value; self.ox = p['grid.origin_x'].value; self.oy = p['grid.origin_y'].value
        self.z_hit = p['sensor_model.z_hit'].value; self.z_rand = p['sensor_model.z_rand'].value
        self.sigma_hit = p['sensor_model.sigma_hit'].value; self.max_range = p['sensor_model.max_range'].value
        self.skip_beams = p['update.skip_beams'].value; self.down_th = p['update.downsample_theta'].value

        self.map = None
        self.scan = None
        self.last_odom = None
        self.belief = np.ones((self.ny, self.nx, self.nth), dtype=np.float32)
        self.belief /= self.belief.sum()
        xs, Kxy, thetas, Kth = motion_kernel(self.nth,
                                             p['motion_model.trans_sigma'].value,
                                             p['motion_model.rot_sigma'].value,
                                             p['motion_model.trans_rot_sigma'].value,
                                             self.res)
        self.mm = (xs, Kxy, thetas, Kth)
        self.br = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(PoseStamped, 'markov_pose', 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 100)
        self.timer = self.create_timer(1.0/float(p['publish_rate_hz'].value), self.publish_pose)

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg

    def scan_cb(self, msg: LaserScan):
        self.scan = msg
        self.update()

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        curr = (x, y, yaw)
        if self.last_odom is not None:
            dtrans, drot1, drot2 = odom_to_delta(self.last_odom, curr)
            self.predict(dtrans, drot1 + drot2)
        self.last_odom = curr

    def predict(self, dtrans, dtheta):
        # shift belief approximately by translating in heading direction bins and rotating heading
        xs, Kxy, thetas, Kth = self.mm
        # Rotate dimension
        rot_bins = int(round((dtheta % (2*math.pi)) / (2*math.pi) * self.nth))
        self.belief = np.roll(self.belief, rot_bins, axis=2)
        # Translate by convolving x/y per theta slice
        new = np.zeros_like(self.belief)
        for th_bin in range(0, self.nth, self.down_th):
            th = th_bin * (2*math.pi/self.nth)
            dx_cells = int(round((dtrans * math.cos(th))/self.res))
            dy_cells = int(round((dtrans * math.sin(th))/self.res))
            shifted = np.roll(self.belief[:,:,th_bin], (dy_cells, dx_cells), axis=(0,1))
            new[:,:,th_bin] = shifted
        # blur to model noise
        self.belief = 0.5*self.belief + 0.5*new
        self.belief += 1e-12
        self.belief /= self.belief.sum()

    def update(self):
        if self.map is None or self.scan is None:
            return
        grid = np.array(self.map.data, dtype=np.int16).reshape(self.map.info.height, self.map.info.width)
        angles = np.arange(self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment*self.skip_beams)
        z_obs = np.array(self.scan.ranges[::self.skip_beams])
        w = np.zeros_like(self.belief)
        for th_bin in range(0, self.nth, self.down_th):
            th = th_bin * (2*math.pi/self.nth)
            # For efficiency, evaluate for a sparse set of (y,x) cells
            step = 2
            for gy in range(0, self.ny, step):
                for gx in range(0, self.nx, step):
                    x, y = (gx*self.res + self.ox + 0.5*self.res,
                            gy*self.res + self.oy + 0.5*self.res)
                    z_exp = expected_ranges_from_map(grid, self.ox, self.oy, self.res, x, y, th, angles, self.max_range)
                    pz = likelihood_beam_model(z_exp, z_obs, self.z_hit, self.z_rand, self.sigma_hit, self.max_range)
                    w[gy, gx, th_bin] = np.prod(pz + 1e-6)
            # fill skipped cells by local averaging
            if step > 1:
                from scipy.ndimage import zoom
                w[:,:,th_bin] = zoom(w[:,:,th_bin], step, order=1, prefilter=False, grid_mode=True, mode='nearest')[:self.ny,:self.nx]
        self.belief *= w + 1e-9
        self.belief += 1e-12
        self.belief /= self.belief.sum()

    def publish_pose(self):
        # MAP estimate
        idx = np.unravel_index(np.argmax(self.belief), self.belief.shape)
        gy, gx, th_bin = idx
        x, y = gx*self.res + self.ox + 0.5*self.res, gy*self.res + self.oy + 0.5*self.res
        th = th_bin * (2*math.pi/self.nth)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_map
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z = math.sin(th/2.0)
        ps.pose.orientation.w = math.cos(th/2.0)
        self.pub_pose.publish(ps)

        # map->base transform (for visualization)
        t = TransformStamped()
        t.header = ps.header
        t.child_frame_id = self.frame_base
        t.transform.translation.x = x; t.transform.translation.y = y
        t.transform.rotation = ps.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = MarkovLocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()
