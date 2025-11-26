import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthDecompressor(Node):
    def __init__(self):
        super().__init__('depth_decompressor_node')
        
        # Subscribe ke topik compressed
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/aligned_depth_to_color/image_raw/compressedDepth',
            self.listener_callback,
            10)
        
        # Publisher topik raw baru
        self.publisher_ = self.create_publisher(Image, '/camera/depth_decompressed', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Depth Decompressor Node Started (Manual Decode Mode)...')

    def listener_callback(self, msg):
        try:
            # 1. Ubah data message menjadi Numpy Array uint8
            # Ini langkah yang sebelumnya gagal di cv_bridge
            np_arr = np.array(msg.data, dtype=np.uint8)

            # 2. Handle format 'compressedDepth'
            # Format ini memiliki header 12 byte sebelum data PNG dimulai.
            # Byte ke-13 (index 12) biasanya bernilai 137 (0x89) header PNG.
            if 'compressedDepth' in msg.format:
                # Pastikan data cukup panjang
                if len(np_arr) > 12:
                    # Buang 12 byte pertama, ambil sisanya (PNG murni)
                    np_arr = np_arr[12:]
                else:
                    self.get_logger().error("Data too short for compressedDepth")
                    return

            # 3. Dekompresi manual menggunakan OpenCV
            # IMREAD_UNCHANGED sangat PENTING agar tetap 16-bit (tidak dikonversi ke 8-bit)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

            if cv_image is None:
                self.get_logger().error("Failed to decode image via cv2.imdecode")
                return

            # 4. Convert balik ke ROS Image Message (Raw)
            # Encoding 'mono16' atau '16UC1' sama saja untuk depth
            raw_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="16UC1")
            
            # Salin header asli agar sinkronisasi waktu tetap terjaga
            raw_msg.header = msg.header
            
            # 5. Publish
            self.publisher_.publish(raw_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthDecompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()