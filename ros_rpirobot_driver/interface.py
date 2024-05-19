"""Robot control interface."""
import rclpy
from rclpy.node import Node
import serial
from robot_interfaces.msg import Distance, Velocity


def hexstr_to_int(string_data):
    """Convert a string of hexadecimal data into a signed integer."""
    number = int(string_data, 16)
    if number > 0x7fff:
        number -= 0x10000
    return number


class ControlInterface(Node):
    """Robot control interface node."""

    def __init__(self):
        """Create robot control interface node."""
        super().__init__("robot_control_interface")

        # Publishers for distance and velocity
        self.pub_distance = self.create_publisher(Distance, 'distance', 10)
        self.pub_velocity = self.create_publisher(Velocity, 'velocity', 10)

        # TODO: use module parameter
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.get_logger().info('%s %s' % (serial_port, baud_rate))
        self.__interface = serial.Serial(serial_port, baud_rate)

        # Update timer for data fetch, and speed keepalive
        timer_period = 0.5  # seconds
        self.status_timer = self.create_timer(
            timer_period,
            self.status_timer_callback
        )

    def __del__(self):
        """Destructs robot control interface node."""
        self.__interface.close()

    def __parse_motor_data(self, response_string):
        self.get_logger().info("right: %s" % response_string[0:4])
        self.get_logger().info("left: %s" % response_string[4:8])
        return (
            hexstr_to_int(response_string[0:4]),
            hexstr_to_int(response_string[4:8])
        )

    def __parse_sensor_data(self, response_string):
        return (
            hexstr_to_int(response_string[0:4]),
            hexstr_to_int(response_string[4:8]),
            hexstr_to_int(response_string[8:12]),
            hexstr_to_int(response_string[12:16])
        )

    def status_timer_callback(self):
        """Execute hardware status read."""
        # get sensor right left forward backward
        self.get_logger().info("start")
        self.__interface.write(str.encode('gsrlfb\r'))
        self.__interface.flush()

        self.get_logger().info("req sent")
        ack = self.__interface.readline().decode('utf-8')
        self.get_logger().info("line read")

        if ack != "k\r\n":
            self.get_logger().error(
                'No acknowledge received, instead i had: "%s"' %
                ack
                )
            return

        self.get_logger().info("ack found")
        answer = self.__interface.readline().decode('utf-8')
        dist = Distance()
        (dist.right,
         dist.left,
         dist.forward,
         dist.backward) = self.__parse_sensor_data(answer)
        self.get_logger().info("dist found")

        done = self.__interface.readline().decode('utf-8')
        if done != "n\r\n":
            self.get_logger().error(
                'no done received, instead i had: "%s"' %
                done
                )
            return
        self.get_logger().info("done found")

        # get motor speeds right left
        self.__interface.write(str.encode('gvrl\r'))
        self.__interface.flush()
        ack = self.__interface.readline().decode('utf-8')
        if ack != "k\r\n":
            self.get_logger().error(
                'No acknowledge received, instead i had: "%s"' %
                ack
                )
            return

        answer = self.__interface.readline().decode('utf-8')
        vel = Velocity()
        self.get_logger().info('received vel: %s' % answer)
        (vel.right, vel.left) = self.__parse_motor_data(answer)

        done = self.__interface.readline().decode('utf-8')
        if done != "n\r\n":
            self.get_logger().error(
                'no done received, instead i had: "%s"' %
                done
                )
            return

        # publish motor and sensor data
        self.pub_distance.publish(dist)
        self.pub_velocity.publish(vel)


def main():
    """Start Control interface node."""
    rclpy.init()

    control_interface = ControlInterface()

    rclpy.spin(control_interface)


if __name__ == '__main__':
    main()
