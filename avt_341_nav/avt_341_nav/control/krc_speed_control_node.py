#!/usr/bin/env python3
#################################################################################################
# krc_speed_control_node.py                                                                     #
# Author: Evan Vandermate (evanderm@mtu.edu)                                                    #
# --------------------------------------------------------------------------------------------- #
# Main interface for controlling MRZR motion.                                                   #
# Controls throttle, brake, and steering to maintain requested longitudinal speed and yaw rate. #
#                                                                                               #
# [Subscribers]                                                                                 #
#   ~avt_341/velocity_rqst (geometry_msgs/msg/Twist): Requested longitudinal speed (linear.x)   #
#                                       [m/s] and wheel angle (angular.z) [rad], i.e. the       #
#                                       speed_control_node velocity-mode output                 #
#   ~avt_341/forward_speed (std_msgs/msg/Float64): Longitudinal speed feedback [m/s]            #
#                                                                                               #
# [Publishers]                                                                                  #
#   ~avt_341/cmd_vel (geometry_msgs/msg/Twist): Throttle (linear.x), brake (linear.y)           #
#                                       and wheel angle (angular.z) [rad] request               #
#################################################################################################
# ROS Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

from avt_341_nav.control.pid import PID
from avt_341_nav.control.topic_watchdog import TopicWatchdog
from avt_341_nav.krc_speed_control_parameters import avt_341_nav_params_krc_speed_control


class VelocityControl(Node):
    def __init__(self):
        # Initialize node
        super().__init__('velocity_control')

        # Params
        self.param_listener = avt_341_nav_params_krc_speed_control.ParamsListener(self)
        self.params = self.param_listener.get_params()

        # Subscribers
        self.vel_cmd = TopicWatchdog(self, 'avt_341/velocity_rqst', Twist)
        self.current_speed = TopicWatchdog(self, 'avt_341/forward_speed', Float64)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'avt_341/cmd_vel', 1)

        # Class Vars
        self.pid = self.make_pid()

        # Timers
        self.create_timer(1.0/self.params.control_rate, self.control_loop)  # Control loop
        self.create_timer(0.2, self.update_dynamic_params)  # Param update loop

    def make_pid(self):
        return PID(p=self.params.throttle_kp, i=self.params.throttle_ki, d=self.params.throttle_kd,
                   max_i=self.params.max_integral)

    # Map throttle values to [min_throttle, max_throttle] range
    def map_throttle(self, thr):
        thr_out = max(0.0, min(1.0, thr))
        thr_out = self.params.min_throttle + thr_out * (self.params.max_throttle - self.params.min_throttle)
        return thr_out

    def control_loop(self):
        if not self.current_speed.is_valid():
            if self.params.verbose:
                self.get_logger().warning("Speed feedback not received yet")
            return  # No speed feedback yet
        if not self.vel_cmd.is_valid():
            if self.params.verbose:
                self.get_logger().warning("Velocity commands not received yet")
            return  # No velocity commands received

        # Update PID controller and compute throttle output
        vel_cmd_msg = self.vel_cmd.value()
        throttle = 0.0
        brake = 0.0
        if (vel_cmd_msg.linear.x > self.params.stopped_speed):
            current_speed_msg = self.current_speed.value()
            throttle = self.pid.update(vel_cmd_msg.linear.x, current_speed_msg.data)
            throttle = self.map_throttle(throttle)
        else:
            brake = self.params.stopped_brake

        # Publish throttle, brake and steering angle request
        cmd_msg = Twist()
        cmd_msg.linear.x = throttle
        cmd_msg.linear.y = brake
        cmd_msg.angular.z = vel_cmd_msg.angular.z
        self.cmd_pub.publish(cmd_msg)

    def update_dynamic_params(self):
        if not self.param_listener.is_old(self.params):
            return
        self.params = self.param_listener.get_params()

        if self.params.throttle_kp != self.pid.Kp or \
                self.params.throttle_ki != self.pid.Ki or \
                self.params.throttle_kd != self.pid.Kd or \
                self.params.max_integral != self.pid.max_i:
            self.pid = self.make_pid()  # Update PID Controller

def main(args=None):
    rclpy.init(args=args)
    node = VelocityControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
