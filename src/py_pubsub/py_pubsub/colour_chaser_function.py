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

# A colour chasing function by Carter Stone
# running successfully this file should allow the Limo robot within the gazebo 
# simulation to rotate until it finds a colour within the specified parameters (green and red)
# locte the centre of the colour and move toward it pushing it to the wall for 5 seconds
# after pushiong it should rotate to find a new target
# Potential Improvements: I experienced some technical problems whilst woking on this
# but with more time I would like to add path-finding capability to navigate around 
# static objects in the robots way.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # Convert between ROS and OpenCV
import numpy as np
import cv2 
import time

# Edited code from:
# github.com/LCAS/teaching/src/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/colour_chaser.py
class ColourChaser(Node):

    def __init__(self):
        super().__init__('colour_chaser') # Node name
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1) # Publish data type, topic and queue length
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1) # Subscription data type, topic, call trigger and queue
        self.get_logger().info("ColourChaser node has started")
        self.tw = Twist()
        self.br = CvBridge()

        # variables
        self.rotating = False
        self.rotateTime = None
        self.pushing = False
        self.pushTime = None
        print("colour chaser initialised")

    def camera_callback(self, data):
        now = time.time()

        # If we are in rotate mode rotate for 3 seconds
        if self.rotating == True:
            if now - self.rotateTime < 3.0:
                self.tw.linear.x = 0.0
                self.tw.angular.z = 0.5
                self.pub_cmd_vel.publish(self.tw)
                return
            else:
                self.rotating = False # return to usual search
        
        # if we are in push mode push for 5 seconds
        if self.pushing == True:
            if now - self.pushTime < 5.0:
                self.tw.linear.x = 0.2
                self.tw.angular.z = 0.0
                self.pub_cmd_vel.publish(self.tw)
            else:
                self.pushing = False
                self.rotating = True
                self.rotateTime = now
                return

        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Colour masks
        # Modified code from:
        # https://answers.opencv.org/question/237367/color-threshholding-only-outputs-edge-for-green-color/
        # By adding red boundaries and mask
        ########################################
        # Identifies the upper and lower thresholdes of green
        lower_green = np.array([80, 140, 110]) 
        upper_green = np.array([90, 160, 125]) 

        # Identifies the upper and lower thresholds of red
        lower_red = np.array([0, 70, 50]) 
        upper_red = np.array([10, 255, 255]) 

        # isolates colours within that range
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
        ########################################

        # Merge colour masks
        # Modified code from
        # https://stackoverflow.com/questions/66515023/how-to-merge-two-bitmasks-in-opencv-with-different-colors
        # By changing the variable names
        combined_mask = cv2.bitwise_or(red_mask, green_mask)
        ####################################################

        contours = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        # Selects the biggest contour, finds the center and reacts to it
        if len (contours) > 0:
            # some modified code from https://learnopencv.com/tag/cv2-moments/
            M = cv2.moments(contours[0])
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])  # Target center
                cy = int(M["m01"] / M["m00"])

                # Calculating distance from centre to target
                width = cv_image.shape[1]
                center_x = width // 2
                dist_x = cx - center_x
                if (dist_x > 100): # If target is more than 100 px from the center
                    self.tw.angular.z = -0.2 # Turn right (rotating mode not needed here)
                    self.tw.linear.x = 0.0
                    print("Right...")
                elif (dist_x < -100):
                    self.tw.angular.z = 0.2 # Turn left
                    self.tw.linear.x = 0.0
                    print("Left...")
                else:  # target is within range
                    self.tw.linear.x = 0.1  # Move forward (start pushing timer)
                    self.tw.angular.z = 0.0
                    self.pushing = True 
                    self.pushTime = now
                    print("Forward...")
            else:
                cx, cy = 0, 0

        else:
            self.tw.linear.x = 0.0
            self.tw.angular.z = 0.3  # Rotate to find target
            self.get_logger().info("No target found. Rotating...")

        self.pub_cmd_vel.publish(self.tw)

        #img_resized = cv2.resize(image_contours, (0,0), fx=0.4, fy=0.4) # Reduce image size

def main(args=None):
    print('Starting colour_chaser.py')
    rclpy.init(args=args)
    node = ColourChaser()
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
