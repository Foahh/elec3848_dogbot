import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Range
from threading import Thread
import socket
import time, copy
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

DEFAULT_LINEAR_VELOCITY = 0.25
DEFAULT_ANGULAR_VELOCITY = -0.4


class ServerPublisher(Node):
    def __init__(self, server_socket):
        super().__init__("DogbotServerNode")
        self.server_socket = server_socket

        self.twist_stamped = TwistStamped()
        self.servo_position = Float64MultiArray()

        self.twist_publisher = self.create_publisher(
            TwistStamped, "/dogbot_base_controller/cmd_vel", 10
        )
        self.servo_publisher = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )

        self.sonar_data = -1.0 # meters
        self.sonar_listener = self.create_subscription(
            Range,
            "/range_sensor_broadcaster/range",
            self.sonar_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_x = 0.0  # meters
        self.current_y = 0.0  # meters
        self.current_z = 0.0  # radians

        self.forearm = 90.0
        self.gripper = 30.0

        self.forearm_down = 60.0  # 53
        self.forearm_up = 140.0  # ??
        self.gripper_close = 95.0
        self.gripper_open = 30.0

        self.rotate_period = 0.6
        self.rotate_angle = DEFAULT_ANGULAR_VELOCITY
        self.heading_period = 0.3

        self.area = 0
        self.Xoffset = 0
        self.confidence = 0

        self.state = "stop"
        self.cmds = []
        self.detected = False
        self.tstamp = time.time()
        self.prev_dist = []
        self.heading = False
        self.dist_threshold = 0.13
        self.dist_len_threshold = 100
    
    def sonar_callback(self, msg):
        self.sonar_data = msg.range
        # self.get_logger().info(f"Sonar data: {self.sonar_data}")

    def update_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            self.current_x = transform.transform.translation.x
            self.current_y = transform.transform.translation.y
            roll, pitch, yaw = euler_from_quaternion(
                (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                )
            )
            self.current_z = yaw
        except TransformException as e:
            self.get_logger().error(e)

    def send_pose(self, client_socket):
        self.update_tf()
        self.get_logger().info(
            f"Current pose: ({self.current_x}, {self.current_y}, {self.current_z})"
        )
        self.__send(
            client_socket, f"pose,{self.current_x},{self.current_y},{self.current_z}"
        )

    def forward(self):
        timestamp = self.get_clock().now().to_msg()
        self.twist_stamped.header.stamp = timestamp
        self.twist_stamped.twist.linear.x = DEFAULT_LINEAR_VELOCITY
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    def backward(self):
        timestamp = self.get_clock().now().to_msg()
        self.twist_stamped.header.stamp = timestamp
        self.twist_stamped.twist.linear.x = -DEFAULT_LINEAR_VELOCITY
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    def left(self):
        timestamp = self.get_clock().now().to_msg()
        self.twist_stamped.header.stamp = timestamp
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = DEFAULT_LINEAR_VELOCITY
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    def right(self):
        timestamp = self.get_clock().now().to_msg()
        self.twist_stamped.header.stamp = timestamp
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = -DEFAULT_LINEAR_VELOCITY
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    def spin_cw(self):
        timestamp = self.get_clock().now().to_msg()
        self.twist_stamped.header.stamp = timestamp
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = DEFAULT_ANGULAR_VELOCITY
        self.twist_publisher.publish(self.twist_stamped)

    def spin_ccw(self):
        timestamp = self.get_clock().now().to_msg()
        self.twist_stamped.header.stamp = timestamp
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = -DEFAULT_ANGULAR_VELOCITY
        self.twist_publisher.publish(self.twist_stamped)

    def stop(self):
        timestamp = self.get_clock().now().to_msg()
        self.twist_stamped.header.stamp = timestamp
        self.state = "stop"
        self.twist_stamped.twist.linear.x = 0.0
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist_stamped)

    def ser_wheel_velocity(self, linear_x, linear_y, angular_z):
        timestamp = self.get_clock().now().to_msg()
        self.twist_stamped.header.stamp = timestamp
        self.twist_stamped.twist.linear.x = linear_x
        self.twist_stamped.twist.linear.y = linear_y
        self.twist_stamped.twist.angular.z = angular_z
        self.twist_publisher.publish(self.twist_stamped)

    def set_servo_position(self, forearm, gripper):
        self.servo_position.data = (forearm, gripper)
        self.servo_publisher.publish(self.servo_position)

    def cmd_handler(self):
        while True:
            try:
                match self.state:
                    case "r_cw":
                        self.prev_dist = []
                        self.ser_wheel_velocity(0.0, 0.0, DEFAULT_ANGULAR_VELOCITY - 0.4)
                        time.sleep(self.rotate_period)
                        self.stop()
                        self.state = "stop"
                        time.sleep(0.3)
                        # if time.time() - self.tstamp > self.rotate_period: # or self.detected == False:
                        #     self.stop()
                        #     self.state = "stop"
                        # continue
                    case "r_ccw":
                        self.prev_dist = []
                        self.ser_wheel_velocity(0.0, 0.0, -DEFAULT_ANGULAR_VELOCITY)
                        time.sleep(self.rotate_period)
                        self.stop()
                        self.state = "stop"
                        time.sleep(0.3)
                        # if time.time() - self.tstamp > self.rotate_period: # or self.detected == False:
                        #     self.stop()
                        #     self.state = "stop"
                        # continue
                    case "heading":
                        if self.sonar_data >= self.dist_threshold and self.sonar_data < 1:
                            self.state = "heading_target"
                            self.prev_dist = []
                            self.tstamp = time.time()
                            self.ser_wheel_velocity(0.15, 0.0, 0.0)
                    case "heading_target":
                        self.prev_dist = []
                        if time.time() - self.tstamp > self.heading_period: # or self.detected == False:
                            self.stop()
                            self.state = "stop"
                    case "grab":
                        self.set_servo_position(self.forearm_down, self.gripper_open)
                        self.state = "grab_2"
                        self.tstamp = time.time()
                        continue
                    case "grab_2":
                        if time.time() - self.tstamp > 2:
                            self.state = "grab_3"
                            self.set_servo_position(self.forearm_down, self.gripper_close)
                            self.tstamp = time.time()
                        continue
                    case "grab_3":
                        if time.time() - self.tstamp > 2:
                            self.state = "grab_4"
                            self.set_servo_position(self.forearm_up, self.gripper_close)
                            self.tstamp = time.time()
                        continue
                    case "grab_4":
                        if time.time() - self.tstamp > 2:
                            self.state = "stop"
                            self.set_servo_position(self.forearm, self.gripper)
                            self.ser_wheel_velocity(-0.7, 0.0, 0.0)
                        continue
                    case "stop":
                        self.set_servo_position(self.forearm, self.gripper)
                        if self.sonar_data > 8:
                            self.set_servo_position(self.forearm_down, self.gripper_close)
                            time.sleep(1)
                        if self.sonar_data < 0.25 and self.sonar_data >= self.dist_threshold:
                            self.prev_dist = []
                            self.state = "heading_target"
                            self.tstamp = time.time()
                            self.ser_wheel_velocity(0.15, 0.0, 0.0)
                        elif len(self.prev_dist) == self.dist_len_threshold:
                            self.state = "grab"
                            self.prev_dist = []
                            continue
                        elif self.sonar_data < self.dist_threshold:
                            self.prev_dist.append(self.sonar_data)
                        elif self.sonar_data > self.dist_threshold and len(self.prev_dist) != 0:
                            self.prev_dist.pop(-1)
                        elif self.tstamp - time.time() > 4:
                            self.ser_wheel_velocity(-0.5, 0.0, 0.0)
                            time.sleep(1)
                            if self.detected == True:
                                break
                            time.sleep(1)
                            self.ser_wheel_velocity(0.0, 0.0, 2.0)
                            if self.detected == True:
                                break
                            time.sleep(1)
                            self.ser_wheel_velocity(0.0, 0.0, -3.5)
                            if self.detected == True:
                                break
                            self.set_servo_position(self.forearm_down, self.gripper_close)
                            time.sleep(1)
                            self.set_servo_position(self.forearm, self.gripper)
                            self.tstamp = time.time()
            except Exception as e:
                self.get_logger().info(e)

            if self.cmds == []:
                continue
            else:
                cmd, *args = self.cmds

            try:
                match cmd:
                    case "pose":
                        if len(args) >= 2:
                            forearm = args[0]
                            gripper = args[1]
                            self.set_servo_position(forearm, gripper)
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
                        self.forearm, self.gripper = map(float, args)
                    case "crusing":
                        self.state = "crusing"
                    case "approaching":
                        self.state = "approaching"
                    case "dist_thre":
                        if len(args) >= 1:
                            self.dist_threshold = float(args[0])
                    case "dist_len_thre":
                        if len(args) >= 1:
                            self.dist_len_threshold = float(args[0])
                    case "detected":
                        self.detected = True
                        if len(args) >= 4:
                            self.area = float(args[0])
                            self.Xoffset = float(args[1])
                            self.Yoffset = float(args[2])
                            self.confidence = float(args[3])
                    case "undetected":
                        self.detected = False
                    case "r_cw":
                        # if self.state == "r_cw":
                        #     pass
                        if len(args) >= 2:
                            self.rotate_angle = float(args[0])
                            self.rotate_period = float(args[1])
                            self.ser_wheel_velocity(0.0, 0.0, self.rotate_angle)
                            self.tstamp = time.time()
                        else:
                            self.ser_wheel_velocity(0.0, 0.0, DEFAULT_ANGULAR_VELOCITY)
                        self.state = "r_cw"
                    case "r_ccw":
                        # if self.state == "r_ccw":
                        #     pass
                        if len(args) >= 2:
                            self.rotate_angle = float(args[0])
                            self.rotate_period = float(args[1])
                            self.ser_wheel_velocity(0.0, 0.0, -self.rotate_angle)
                            self.tstamp = time.time()
                        else:
                            self.ser_wheel_velocity(0.0, 0.0, -DEFAULT_ANGULAR_VELOCITY)
                        self.state = "r_ccw"
                    case "heading_target":
                        if self.state == "heading":
                            pass
                        else:
                            self.ser_wheel_velocity(0.15, 0.0, 0.0)
                            self.state = "heading"
                            self.tstamp = time.time()
                    case "grab":
                        self.state = "grab"
                    # case _:
                    #     self.stop()
                    #     if cmd == "echoback":
                    #         break
                    #     self.get_logger().error(f"Invalid command: {cmd}.")
            except ValueError:
                self.get_logger().error(f"Invalid parameters: {args}. Caused by command: {cmd}.")
            except TypeError as e:
                self.get_logger().error(e)
                # This exception error could not be solved. It's weird.

    def __send(self, client_addr, msg) -> None:
        self.server_socket.sendto(msg.encode(), client_addr)
        # try:
        #     data = client_socket.recv(1024).decode()
        #     print(data)
        # except:
        #     pass
        return

    def recv_handler(self) -> None:
        while True:
            data_buffer, client_addr = self.server_socket.recvfrom(1024)
            if not data_buffer:
                self.get_logger().error("Connection is broken!")
                self.get_logger().info("Waiting for connection...")
                # client_socket.close()
                # client_socket, _ = self.server_socket.accept()
            recv_data = data_buffer.decode("utf-8")
            new_msg_handler = Thread(
                target=self.msg_handler,
                args=(
                    recv_data,
                    client_addr,
                ),
            )
            new_msg_handler.start()

    def msg_handler(self, recv_data, client_addr) -> None:
        if "echoback" in recv_data:
            s = f"State: {self.state}\nDist:{self.sonar_data}\n"
            try:
                if self.detected == True:
                    s += f"Area:{self.area}\nOffset:{self.Xoffset}\nCon:{self.confidence}\n"
            except:
                pass
            self.__send(client_addr, s)
        elif recv_data:
            if "detected" not in recv_data:
                self.get_logger().info(f"Received: {recv_data}")
            if "forcestop" in recv_data:
                self.state = "stop"
            else:
                cmds = []
                for cmd in recv_data.split("\n")[0].split(","):
                    cmds.append(cmd.strip(' '))
                self.cmds = copy.deepcopy(cmds)

def Nodes(node) -> None:
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("0.0.0.0", 8080))
    # server_socket.listen(5)
    node = ServerPublisher(server_socket)
    node.get_logger().info("Waiting for connection...")

    nodes_thread = Thread(target=Nodes, args=(node,))
    nodes_thread.start()

    handler_thread = Thread(target=node.cmd_handler)
    handler_thread.start()

    try:
        while True:
            # client_socket, _ = server_socket.accept()
            client_thread = Thread(target=node.recv_handler)
            client_thread.start()
    except KeyboardInterrupt:
        exit(0)