# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # Convert between ROS and OpenCV
import numpy as np
import cv2

# Edited code from:
# github.com/LCAS/teaching/src/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/colour_chaser.py
class ColourChaser(Node):

    def __init__(self):
        super().__init__('colour_chaser')
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        print("Publisher created")
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)
        print("Subscription created")
        self.get_logger().info("ColourChaser node has started")
        self.tw = Twist()
        self.br = CvBridge()

    def camera_callback(self, data):
        
        cv2.namedWindow("ImageWindow", 1)
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Colour masks
        # Edited code from:
        # https://answers.opencv.org/question/237367/color-threshholding-only-outputs-edge-for-green-color/
        ########################################
        # Identifies the upper and lower thresholdes of green
        lower_green = np.array([80, 140, 110]) 
        upper_green = np.array([90, 160, 125]) 

        # Identifies the upper and lower thresholds of red
        lower_red = np.array([0, 70, 50]) 
        upper_red = np.array([10, 255, 255]) 

        # isolates colours within that range
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        #red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
        ########################################

        # Sorts by area and keeps the biggest
        contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        image_contours = cv2.drawContours(cv_image, contours, 0, (0, 100, 100), 20)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                width = cv_image.shape[1]
                center_x = width // 2
                dist_x = cx - center_x

                self.tw.linear.x = 0.1  # Move forward
                self.tw.angular.z = -float(dist_x) / 300.0  # Turn to center target
                self.get_logger().info(f"Tracking target at x={cx}, turning with z={self.tw.angular.z:.2f}")
            else:
                self.tw.linear.x = 0.0
                self.tw.angular.z = 0.3  # Rotate to search
        else:
            self.tw.linear.x = 0.0
            self.tw.angular.z = 0.3  # Rotate in place to find target
            self.get_logger().info("No target found. Rotating...")

        self.pub_cmd_vel.publish(self.tw)

def main(args=None):
    print('Starting colour_chaser.py')
    #cv2.startWindowThread()

    rclpy.init(args=args)
    node = ColourChaser()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
