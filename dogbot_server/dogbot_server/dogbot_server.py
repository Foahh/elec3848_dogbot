import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from threading import Thread
import socket
import time
from functools import wraps
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


DEFAULT_LINEAR_VELOCITY = 0.25
DEFAULT_ANGULAR_VELOCITY = 0.5


def twist_add_header(func):
    @wraps(func)
    def _impl(self, func):
        timestamp = self.get_clock().now().to_msg()
        self.twist_stamped.header.stamp = timestamp
        func()

    return _impl


class ServerPublisher(Node):
    def __init__(self, server_socket):
        super().__init__("DogbotServerNode")
        self.server_socket = server_socket

        self.twist_stamped = TwistStamped()
        self.servo_position = Float64MultiArray()
        self.imu_data = Imu()

        self.twist_publisher = self.create_publisher(
            TwistStamped, "dogbot_base_controller", 10
        )
        self.servo_publisher = self.create_publisher(
            Float64MultiArray, "dogbot_servo_controller", 10
        )
        self.imu_subscriber = self.create_subscription(
            Imu, "imu/data", self.imu_callback, 10
        )

        self.data = ""
        self.state = "stop"

    def imu_callback(self, msg):
        self.imu_data = msg

    @twist_add_header
    def forward(self):
        self.twist_stamped.twist.linear.x = DEFAULT_LINEAR_VELOCITY
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @twist_add_header
    def backward(self):
        self.twist_stamped.twist.linear.x = -DEFAULT_LINEAR_VELOCITY
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @twist_add_header
    def left(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = DEFAULT_LINEAR_VELOCITY
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @twist_add_header
    def right(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = -DEFAULT_LINEAR_VELOCITY
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @twist_add_header
    def spin_cw(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = DEFAULT_ANGULAR_VELOCITY
        self.twist_publisher.publish(self.twist_stamped)

    @twist_add_header
    def spin_ccw(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = -DEFAULT_ANGULAR_VELOCITY
        self.twist_publisher.publish(self.twist_stamped)

    @twist_add_header
    def stop(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @twist_add_header
    def ser_wheel_velocity(self, linear_x, linear_y, angular_z):
        self.twist_stamped.twist.linear.x = linear_x
        self.twist_stamped.twist.linear.y = linear_y
        self.twist_stamped.twist.angular.z = angular_z
        self.twist_publisher.publish(self.twist_stamped)

    def set_servo_position(self, forearm, gripper):
        self.servo_position.data = (forearm, gripper)
        self.servo_publisher.publish(self.servo_position)

    def on_receive(self, client_socket):
        while True:
            data_buffer = client_socket.recv(1024)

            if not data_buffer:
                self.get_logger().error("Connection is broken!")
                self.get_logger().info("Waiting for connection...")
                client_socket.close()
                client_socket, _ = self.server_socket.accept()
                continue

            if self.state in ["r_cw", "r_ccw", "heading_target"]:
                # discard all the coming-in commands before finishing
                if time.time() - self.tstamp < 0.5:
                    continue
                else:
                    self.stop()
                    self.state = "stop"
                    continue

            self.data += data_buffer.decode("utf-8")
            if self.data[-1] != "\n":
                continue

            self.get_logger().info(f"Received: {self.data}")

            try:
                cmd, *args = self.data.strip("\n").split(",")
                match cmd:
                    case "angle":
                        roll, pitch, yaw = euler_from_quaternion(
                            (
                                self.imu_data.orientation.x,
                                self.imu_data.orientation.y,
                                self.imu_data.orientation.z,
                                self.imu_data.orientation.w,
                            )
                        )
                        self.__send(client_socket, f"angle,{roll},{pitch},{yaw}")
                    case "forward":
                        self.forward()
                    case "backward":
                        self.backward()
                    case "left":
                        self.left()
                    case "right":
                        self.right()
                    case "spin_cw":
                        self.spin_cw()
                    case "spin_ccw":
                        self.spin_ccw()
                    case "stop":
                        self.stop()
                    case "velocity":
                        linear_x, linear_y, angular_z = map(float, args)
                        self.ser_wheel_velocity(linear_x, linear_y, angular_z)
                    case "position":
                        forearm, gripper = map(float, args)
                        self.set_servo_position(forearm, gripper)
                    case "crusing":
                        self.__send(client_socket, cmd)
                        self.state = "crusing"
                    case "approaching":
                        self.state = "approaching"
                        self.__send(client_socket, cmd)
                    case "r_cw":
                        # (angle) = map(float, args)
                        self.cmd_vel(0, 0, -1)
                        self.state = "r_cw"
                        self.tstamp = time.time()
                    case "r_ccw":
                        # (angle) = map(float, args)
                        self.cmd_vel(0, 0, 1)
                        self.state = "r_ccw"
                        self.tstamp = time.time()
                    case "heading_target":
                        self.cmd_vel(1, 0, 0)
                        self.state = "heading_target"
                        self.tstamp = time.time()
                    case "grab":
                        forearm_down = 210  # 53
                        forearm_up = 90  # ??
                        gripper_close = 95
                        gripper_open = 30
                        self.cmd_pos(0, 0, 0, forearm, gripper_open)
                        # need to stuck here
                    case _:
                        self.stop()
                        self.get_logger().error(f"Invalid command: {cmd}")
            except ValueError:
                self.get_logger().error(f"Invalid parameters: {self.data}")
            except TypeError as e:
                self.get_logger().error(e)
                # This exception error could not be solved. It's weird.

            self.data = ""

    def __send(self, client_socket, msg) -> object:
        client_socket.sendall(msg.encode())
        data = client_socket.recv(1024).decode()
        print(data)
        return


def Nodes(node) -> None:
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return


def main(args=None):
    rclpy.init(args=args)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("0.0.0.0", 8080))
    server_socket.listen(5)
    node = ServerPublisher(server_socket)
    node.get_logger().info("Waiting for connection...")

    nodes_thread = Thread(target=Nodes, args=(node,))
    nodes_thread.start()

    while True:
        client_socket, _ = server_socket.accept()
        client_thread = Thread(target=node.on_receive, args=(client_socket,))
        client_thread.start()
