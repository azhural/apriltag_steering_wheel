#!/usr/bin/env python3

import rclpy
import tf2_ros
from tf_transformations import euler_from_quaternion
from rclpy.node import Node

from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_timer(0.1, self.timer_cb)

        self.create_subscription(AprilTagDetectionArray, "/apriltag/detections", self.detection, 10)

        self.tag_visible = False

    def timer_cb(self):
        out = Twist()
        if self.tag_visible:
            try:
                trans = self.tf_buffer.lookup_transform("camera", "marker", rclpy.duration.Duration())

                r, p, y = euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
                if r < 0:
                    out.linear.x = 3.14 + r
                else:
                    out.linear.x = -3.14 + r
                out.angular.z = y
            except tf2_ros.TransformException:
                self.tag_visible = False
                out = Twist()

        self.cmd_pub.publish(out)



    def detection(self, msg):
        self.tag_visible = not(not(msg.detections))



def main():
    rclpy.init()
    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
