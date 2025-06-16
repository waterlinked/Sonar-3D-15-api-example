#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import socket
import struct
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__)))
from interface_sonar_api import set_speed, set_acoustics
from sonar_3d_15_protocol_pb2 import RangeImage, BitmapImageGreyscale8
from inspect_sonar_data import handle_packet
from sensor_msgs.msg import Image
import numpy as np
try:
    from rcl_interfaces.msg import SetParametersResult
except ImportError:
    # Fallback for environments where rcl_interfaces is not available
    class SetParametersResult:
        def __init__(self, successful=True):
            self.successful = successful

# Helper to parse RIP1 framing and extract RangeImage protobuf
def parse_rip1_range_image(data):
    if len(data) < 12:
        raise ValueError("Packet too short for RIP1 framing")
    magic, length, msg_type = struct.unpack('<4sII', data[:12])
    if magic != b'RIP1':
        raise ValueError("Invalid RIP1 magic")
    if msg_type != 1:  # 1 = RangeImage
        raise ValueError("Not a RangeImage message")
    pb_data = data[12:12+length]
    range_image = RangeImage()
    range_image.ParseFromString(pb_data)
    return range_image

class Sonar3D15Node(Node):
    def __init__(self):
        super().__init__('sonar_3d_15_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sonar_ip', ''),
                ('speed_of_sound', 1480),
                ('acoustics_enabled', False),
                ('multicast_group', '224.0.0.96'),
                ('multicast_port', 4747),
                ('filter_ip', ''),
            ]
        )
        self.get_logger().info('Sonar 3D-15 ROS2 node started.')
        self.sonar_ip = self.get_parameter('sonar_ip').get_parameter_value().string_value
        self.speed_of_sound = self.get_parameter('speed_of_sound').get_parameter_value().integer_value
        self.acoustics_enabled = self.get_parameter('acoustics_enabled').get_parameter_value().bool_value
        self.multicast_group = self.get_parameter('multicast_group').get_parameter_value().string_value
        self.multicast_port = self.get_parameter('multicast_port').get_parameter_value().integer_value
        self.filter_ip = self.get_parameter('filter_ip').get_parameter_value().string_value

        # Register parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Initial configuration
        self.configure_sonar()

        # Start UDP listening thread
        self.udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        self.udp_thread.start()

        self.image_pub = self.create_publisher(Image, 'sonar/depth_image', 10)
        self.intensity_pub = self.create_publisher(Image, 'sonar/intensity_image', 10)

    def configure_sonar(self):
        if self.sonar_ip:
            try:
                # Use the imported API functions
                resp_speed = set_speed(self.sonar_ip, self.speed_of_sound)
                self.get_logger().info(f"Set speed_of_sound: {resp_speed.status_code}")
                resp_acoustics = set_acoustics(self.sonar_ip, self.acoustics_enabled)
                self.get_logger().info(f"Set acoustics_enabled: {resp_acoustics.status_code}")
            except Exception as e:
                self.get_logger().error(f"Failed to configure sonar: {e}")

    def parameter_callback(self, params):
        success = True
        for param in params:
            if param.name == 'speed_of_sound':
                self.speed_of_sound = param.value
                try:
                    resp = set_speed(self.sonar_ip, self.speed_of_sound)
                    self.get_logger().info(f"Updated speed_of_sound: {resp.status_code}")
                except Exception as e:
                    self.get_logger().error(f"Failed to update speed_of_sound: {e}")
                    success = False
            elif param.name == 'acoustics_enabled':
                self.acoustics_enabled = param.value
                try:
                    resp = set_acoustics(self.sonar_ip, self.acoustics_enabled)
                    self.get_logger().info(f"Updated acoustics_enabled: {resp.status_code}")
                except Exception as e:
                    self.get_logger().error(f"Failed to update acoustics_enabled: {e}")
                    success = False
            elif param.name == 'sonar_ip':
                self.sonar_ip = param.value
                self.configure_sonar()
            elif param.name == 'multicast_group':
                self.multicast_group = param.value
            elif param.name == 'multicast_port':
                self.multicast_port = param.value
            elif param.name == 'filter_ip':
                self.filter_ip = param.value
        return SetParametersResult(successful=success)

    def udp_listener(self):
        multicast_group = self.multicast_group
        port = self.multicast_port
        filter_ip = self.filter_ip
        buffer_size = 65535
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', port))
            group = socket.inet_aton(multicast_group)
            mreq = struct.pack('4sL', group, socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            self.get_logger().info(f"Listening for Sonar 3D-15 UDP packets on {multicast_group}:{port}...")
            if filter_ip:
                self.get_logger().info(f"Filtering packets from IP: {filter_ip}")
            while rclpy.ok():
                data, addr = sock.recvfrom(buffer_size)
                if filter_ip and addr[0] != filter_ip:
                    continue
                self.get_logger().debug(f"Received {len(data)} bytes from {addr}")
                try:
                    result = handle_packet(data)
                    if result is None:
                        continue
                    msg_type, msg_obj = result
                    if msg_type == "RangeImage":
                        # Convert to numpy array (float32)
                        img_np = np.array(msg_obj.image_pixel_data, dtype=np.float32).reshape((msg_obj.height, msg_obj.width))
                        msg = Image()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "sonar"
                        msg.height = msg_obj.height
                        msg.width = msg_obj.width
                        msg.encoding = "32FC1"
                        msg.is_bigendian = False
                        msg.step = img_np.strides[0]
                        msg.data = img_np.tobytes()
                        self.image_pub.publish(msg)
                    elif msg_type == "BitmapImageGreyscale8":
                        # Intensity image as 8UC1
                        img_np = np.array(msg_obj.image_pixel_data, dtype=np.uint8).reshape((msg_obj.height, msg_obj.width))
                        msg = Image()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "sonar"
                        msg.height = msg_obj.height
                        msg.width = msg_obj.width
                        msg.encoding = "8UC1"
                        msg.is_bigendian = False
                        msg.step = img_np.strides[0]
                        msg.data = img_np.tobytes()
                        self.intensity_pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to parse/publish sonar data: {e}")
        except Exception as e:
            self.get_logger().error(f"UDP listener error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Sonar3D15Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
