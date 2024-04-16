import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from dogbot_interfaces.msg import ServoPosition
from threading import Thread
import socket


class ServerPublisherNode(Node):
    def __init__(self, server_socket):
        super().__init__("DogbotServerNode")
        self.server_socket = server_socket
        self.twist_publisher = self.create_publisher(
            TwistStamped, "dogbot_base_controller", 10
        )
        self.twist_stamped = TwistStamped()
        self.servo_position = ServoPosition()
        self.servo_publisher = self.create_publisher(
            ServoPosition, "dogbot_servo_controller", 10
        )
        self.default_vel = 0.25
        self.data = ""

    def __add_header(self, func):
        def warp():
            timestamp = self.get_clock().now().to_msg()
            self.twist_stamped.header.stamp = timestamp
            func()

        return warp

    @__add_header
    def forward(self):
        self.twist_stamped.twist.linear.x = self.default_vel
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @__add_header
    def backward(self):
        self.twist_stamped.twist.linear.x = -self.default_vel
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @__add_header
    def left(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = self.default_vel
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @__add_header
    def right(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = -self.default_vel
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @__add_header
    def turn_left(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = self.default_vel
        self.twist_publisher.publish(self.twist_stamped)

    @__add_header
    def turn_right(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = -self.default_vel
        self.twist_publisher.publish(self.twist_stamped)

    @__add_header
    def stop(self):
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    @__add_header
    def cmd_vel(self, linear_x, linear_y, angular_z):
        self.twist_stamped.twist.linear.x = linear_x
        self.twist_stamped.twist.linear.y = linear_y
        self.twist_stamped.twist.angular.z = angular_z
        self.twist_publisher.publish(self.twist_stamped)

    def cmd_pos(self, pan, tilt, shoulder, forearm, gripper):
        self.servo_position.pan = pan
        self.servo_position.tilt = tilt
        self.servo_position.shoulder = shoulder
        self.servo_position.forearm = forearm
        self.servo_position.gripper = gripper
        self.servo_publisher.publish(self.servo_position)

    def on_receive(
        self,
        client_socket,
    ):
        while True:
            data_buffer = client_socket.recv(1024)

            if not data_buffer:
                self.get_logger().error("Connection is broken!")
                self.get_logger().info("Waiting for connection...")
                client_socket.close()
                client_socket, _ = self.server_socket.accept()
                continue

            self.data += data_buffer.decode("utf-8")
            if self.data[-1] != "\n":
                continue

            self.get_logger().info(f"Received: {self.data}")

            try:
                cmd, *args = self.data.split(",")
                match cmd:
                    case "F":
                        self.forward()
                    case "B":
                        self.backward()
                    case "L":
                        self.left()
                    case "R":
                        self.right()
                    case "<":
                        self.turn_left()
                    case ">":
                        self.turn_right()
                    case "S":
                        self.stop()
                    case "V":
                        linear_x, linear_y, angular_z = map(float, args)
                        self.cmd_vel(linear_x, linear_y, angular_z)
                    case "P":
                        pan, tilt, shoulder, forearm, gripper = map(float, args)
                        self.cmd_pos(pan, tilt, shoulder, forearm, gripper)
                    case _:
                        self.stop()
                        self.get_logger().error(f"Invalid command: {cmd}")
            except ValueError:
                self.get_logger().error(f"Invalid parameters: {self.data}")

            self.data = ""


def main(args=None):
    rclpy.init(args=args)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("0.0.0.0", 8080))
    server_socket.listen(5)
    node = ServerPublisherNode(server_socket)
    node.get_logger().info("Waiting for connection...")

    client_socket, _ = server_socket.accept()
    client_thread = Thread(target=node.on_receive, args=(client_socket, server_socket))
    client_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
