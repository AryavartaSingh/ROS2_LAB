#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        self.target_x = 9.0
        self.target_y = 9.0

        self.pose_ = None

        self.kp_linear = 1.0
        self.ki_linear = 0.005
        self.kd_linear = 0.001
        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        self.kp_angular = 4.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0
        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callback_turtle_pose, 10)

        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def compute_pid(self, kp, ki, kd, error, integral, prev_error):
        integral += error
        derivative = error - prev_error
        output = kp * error + ki * integral + kd * derivative
        return output, integral, error

    def control_loop(self):
        if self.pose_ is None:
            return

        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x ** 2 + dist_y ** 2)
        goal_theta = math.atan2(dist_y, dist_x)

        error_theta = goal_theta - self.pose_.theta
        if error_theta > math.pi:
            error_theta -= 2 * math.pi
        elif error_theta < -math.pi:
            error_theta += 2 * math.pi

        linear_velocity, self.integral_linear, self.prev_error_linear = self.compute_pid(
            self.kp_linear, self.ki_linear, self.kd_linear, distance, self.integral_linear, self.prev_error_linear
        )

        angular_velocity, self.integral_angular, self.prev_error_angular = self.compute_pid(
            self.kp_angular, self.ki_angular, self.kd_angular, error_theta, self.integral_angular, self.prev_error_angular
        )
        msg = Twist()
        if distance > 0.1:
            msg.linear.x = linear_velocity
            msg.angular.z = angular_velocity
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_vel_publisher_.publish(msg)

    def callback_get_distance(self, request, response):
        x = request.loc_x - self.pose_.x
        y = request.loc_y - self.pose_.y
        response.distance = math.sqrt(x * x + y * y)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()