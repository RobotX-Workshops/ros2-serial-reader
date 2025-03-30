import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32


class SerialReader(Node):
    def __init__(self):
        super().__init__("serial_reader", enable_logger_service=True)

        self.get_logger().info("Serial reader node started")
        # Declare parameters with default values
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 9600)

        # Retrieve parameter values
        port = str(self.get_parameter("port").value)  # type: ignore
        baud = int(self.get_parameter("baud").value)  # type: ignore

        self._serial_port = serial.Serial(port, baud)
        topic = "/serial_data"
        self._publisher = self.create_publisher(Int32, topic, 10)
        self.get_logger().info(f"Publishing to topic: {topic}")
        self.get_logger().info(f"Serial port: {port}, Baud rate: {baud}")
        self.get_logger().info("Serial reader node initialized")

    def run(self):
        while rclpy.ok():
            data = self._serial_port.readline().decode().strip()
            if data:
                try:
                    self._publisher.publish(Int32(data=int(data)))
                except ValueError:
                    self.get_logger().error(f"Invalid data: {data}")


def main():
    rclpy.init()
    node = SerialReader()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
