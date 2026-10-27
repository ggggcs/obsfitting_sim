#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class AckermannServo:
    def __init__(self):
        rospy.init_node("servo_commands", anonymous=True)

        # Publishers
        self.pub_left_rear = rospy.Publisher(
            "left_rear_wheel_controller/command", Float64, queue_size=1)
        self.pub_right_rear = rospy.Publisher(
            "right_rear_wheel_controller/command", Float64, queue_size=1)
        self.pub_left_front = rospy.Publisher(
            "left_steer_controller/command", Float64, queue_size=1)
        self.pub_right_front = rospy.Publisher(
            "right_steer_controller/command", Float64, queue_size=1)

        # Vehicle parameters
        self.L = 0.7    # wheelbase
        self.W = 0.6    # track width
        self.r = 0.15   # wheel radius
        self.min_turn_radius = 0.01  # avoid singularity

        # Command inputs
        self.v = 0.0    # forward velocity (m/s)
        self.w = 0.0    # yaw rate (rad/s)

        rospy.Subscriber("cmd_vel", Twist, self.callback)

    def callback(self, msg):
        """Receive v (m/s) and w (rad/s)"""
        self.v = msg.linear.x    # 正确：不应该除以 0.15
        self.w = msg.angular.z   # 正确：w 就是角速度本身

    def run(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            # === 1. 将角速度 w 转换为 Ackermann 前轮角 delta ===
            if abs(self.v) < 1e-6:   # 车停着不能转向
                delta = 0.0
            else:
                # ★ 正确公式：δ = atan(L * w / v)
                delta = math.atan(self.L * self.w / self.v)

            # 限幅（避免转角太大）
            delta = max(min(delta, math.pi/6), -math.pi/6)

            # === 2. Ackermann 前轮左右角计算 ===
            if abs(delta) > 1e-6:
                R = self.L / math.tan(delta)

                # 避免半径太小而导致发散
                sign = 1 if R >= 0 else -1
                R = sign * max(abs(R), self.min_turn_radius)

                # 真实 Ackermann 几何
                phi_left  = math.atan(self.L / (R - self.W/2))
                phi_right = math.atan(self.L / (R + self.W/2))

                # === 3. 左右后轮速度 ===
                v_left  = self.v * (R - self.W/2) / R
                v_right = self.v * (R + self.W/2) / R
            else:
                # 直线
                phi_left = 0.0
                phi_right = 0.0
                v_left = self.v
                v_right = self.v

            # === 4. 将线速度转换成车轮角速度 ===
            omega_left  = v_left / self.r
            omega_right = v_right / self.r

            # Publish
            self.pub_left_front.publish(phi_left)
            self.pub_right_front.publish(phi_right)
            self.pub_left_rear.publish(omega_left)
            self.pub_right_rear.publish(omega_right)

            rate.sleep()


if __name__ == '__main__':
    try:
        node = AckermannServo()
        node.run()
    except rospy.ROSInterruptException:
        pass
