import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32 

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        
        # Declare parameters with default values
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('topic', '/serial_data')
        
        # Retrieve parameter values
        port = str(self.get_parameter('port').value) # type: ignore
        baud = int(self.get_parameter('baud').value) # type: ignore
        topic = str(self.get_parameter('topic').value) # type: ignore
        
        self._serial_port = serial.Serial(port, baud) 
        self._publisher = self.create_publisher(Int32, topic, 10)

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

if __name__ == '__main__':
    main()