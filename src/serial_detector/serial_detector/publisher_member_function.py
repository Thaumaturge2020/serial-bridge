import rclpy
import serial.tools.list_ports
from rclpy.node import Node

from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_device = self.create_publisher(String, 'serial_id', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def publish_serial_ports_info(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # if port.hwid == "USB VID:PID=1366:0105 SER=000020723308 LOCATION=1-9.1:1.0":
            if port.hwid.find("USB")==0 and port.hwid.find("VID")>0 and port.hwid.find("PID")>0 and port.hwid.find("SER")>0 and port.hwid.find("LOCATION")>0:
                print("SER ID:", port.hwid)
                print("Port:", port.device)
                print("Description:", port.description)
                print("HWID:", port.hwid)
                print("===================================")
                msg = String()
                msg.data = port.device
                self.publisher_device.publish(msg)

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1
        self.publish_serial_ports_info()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()