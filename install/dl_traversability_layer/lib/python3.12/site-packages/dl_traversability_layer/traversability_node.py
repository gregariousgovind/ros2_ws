import rclpy, numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from .model import small_unet
import tensorflow as tf

class TraversabilityNode(Node):
    def __init__(self):
        super().__init__('dl_traversability')
        self.declare_parameters('', [
            ('frame_ids.map', 'map'),
            ('topics.map', '/map'), ('topics.scan', '/scan'), ('topics.traversability_map', '/traversability_map'),
            ('model.weights_path', ''), ('model.patch_size', 64), ('model.publish_rate_hz', 2.0),
            ('heuristic.occ_weight', 80), ('heuristic.near_occ_weight', 40), ('heuristic.near_radius', 2),
        ])
        p = self.get_parameters_by_prefix('')
        self.frame_map = p['frame_ids.map'].value
        self.topic_map  = p['topics.map'].value
        self.topic_scan = p['topics.scan'].value
        self.topic_out  = p['topics.traversability_map'].value
        self.patch = p['model.patch_size'].value
        self.rate  = p['model.publish_rate_hz'].value
        self.occ_w = p['heuristic.occ_weight'].value
        self.near_w= p['heuristic.near_occ_weight'].value
        self.near_r= p['heuristic.near_radius'].value
        self.weights_path = p['model.weights_path'].value

        self.map = None
        self.scan = None
        self.pub = self.create_publisher(OccupancyGrid, self.topic_out, 10)
        self.create_subscription(OccupancyGrid, self.topic_map, self.map_cb, 10)
        self.create_subscription(LaserScan, self.topic_scan, self.scan_cb, 10)
        self.timer = self.create_timer(1.0/self.rate, self.publish)

        # Build model (2 channels: occupancy, lidar mask)
        input_shape = (self.patch, self.patch, 2)
        self.model = small_unet(input_shape)
        if self.weights_path:
            try: self.model.load_weights(self.weights_path)
            except Exception as e:
                self.get_logger().warn(f"Failed to load weights: {e}. Using untrained model.")

    def map_cb(self, msg): self.map = msg
    def scan_cb(self, msg): self.scan = msg

    def publish(self):
        if self.map is None:
            return
        H,W = self.map.info.height, self.map.info.width
        res = self.map.info.resolution
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y
        occ = np.array(self.map.data, dtype=np.int16).reshape(H,W)

        # Fallback heuristic if no (or untrained) model:
        if not self.weights_path:
            cost = occ.copy()
            # inflate a bit around obstacles
            if self.near_r > 0:
                from scipy.ndimage import grey_dilation
                near = grey_dilation(occ, size=(self.near_r*2+1, self.near_r*2+1))
                cost = np.clip(cost, 0, 100)
                cost = np.maximum(cost, (near>50)*self.near_w)
            cost = np.clip(cost, 0, 100).astype(np.int16)
        else:
            # Simple global pass: make 2-channel input (normalized occ + radial mask from scan origin if available)
            occ_n = np.clip(occ, 0, 100) / 100.0
            mask = np.zeros_like(occ_n)
            if self.scan is not None:
                # crude circular field-of-view mask
                # (for a real model, fuse per-beam rays; this just provides an example channel)
                mask[:,:] = 1.0
            # resize to patch size for inference (demo)
            import cv2
            inp = np.stack([occ_n, mask], axis=-1).astype(np.float32)
            inp_r = cv2.resize(inp, (self.patch, self.patch), interpolation=cv2.INTER_NEAREST)
            pred = self.model.predict(inp_r[None,...], verbose=0)[0,...,0]
            pred_full = cv2.resize(pred, (W, H), interpolation=cv2.INTER_LINEAR)
            cost = (100.0*(1.0 - pred_full)).astype(np.int16)  # low cost where traversable (pred≈1)

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_map
        out.info = self.map.info
        out.data = cost.flatten().tolist()
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(TraversabilityNode())
    rclpy.shutdown()
