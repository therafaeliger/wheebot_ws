#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')
        self.pub = self.create_publisher(LaserScan, '/scan_for_slam', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)

        # Konfigurasi yang lebih mudah - menggunakan derajat
        self.declare_parameter('ignore_angle_deg', 70.0)  # Sudut yang diabaikan dalam derajat
        self.declare_parameter('ignore_center_deg', 30.0)  # Pusat sudut yang diabaikan dalam derajat
        self.declare_parameter('min_range_ignore', 0.15)  # Jarak minimum yang diabaikan (meter)
        
        # Dapatkan parameter
        ignore_angle_deg = self.get_parameter('ignore_angle_deg').value
        ignore_center_deg = self.get_parameter('ignore_center_deg').value
        self.min_range_ignore = self.get_parameter('min_range_ignore').value
        
        # Konversi ke radian untuk perhitungan internal
        ignore_angle_rad = math.radians(ignore_angle_deg)
        ignore_center_rad = math.radians(ignore_center_deg)
        
        # Hitung batas sudut
        self.angle_ignore_min = ignore_center_rad - ignore_angle_rad / 2
        self.angle_ignore_max = ignore_center_rad + ignore_angle_rad / 2
        
        self.get_logger().info(f'Laser filter configured:')
        self.get_logger().info(f'  Ignoring {ignore_angle_deg}° around {ignore_center_deg}°')
        self.get_logger().info(f'  Angle range: {math.degrees(self.angle_ignore_min):.1f}° to {math.degrees(self.angle_ignore_max):.1f}°')
        self.get_logger().info(f'  Min range ignore: {self.min_range_ignore} m')

    def cb(self, msg: LaserScan):
        out = LaserScan()
        # Salin semua metadata
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        
        # Salin data ranges dan intensities
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities) if msg.intensities else []

        # Filter titik-titik yang tidak diinginkan
        n = len(out.ranges)
        for i in range(n):
            angle = out.angle_min + i * out.angle_increment
            
            # Normalisasi sudut ke rentang [-pi, pi] untuk perbandingan yang benar
            normalized_angle = self.normalize_angle(angle)
            
            # Periksa apakah titik berada dalam sektor yang diabaikan ATAU terlalu dekat
            in_ignore_sector = self.angle_ignore_min <= normalized_angle <= self.angle_ignore_max
            too_close = 0 < out.ranges[i] < self.min_range_ignore
            
            if in_ignore_sector or too_close:
                out.ranges[i] = float('inf')

        self.pub.publish(out)
    
    def normalize_angle(self, angle):
        """Normalisasi sudut ke rentang [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()