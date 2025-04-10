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
        print("Signal recieved")

        self.br = CvBridge()

    def camera_callback(self, data):
        
        cv2.namedWindow("ImageWindow", 1)
    

        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

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
        lower_red = np.array([170, 70, 50]) 
        upper_red = np.array([180, 255, 255]) 
        # isolates colours within that range
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        #red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
        ########################################

        # Sorts by area and keeps the biggest
        contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        image_contours = cv2.drawContours(cv_image, contours, 0, (0, 100, 100), 20)

        if len(contours) > 0:
            #find the centre of the target: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
            M = cv2.moments(contours[0])
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print("Target found. Centering to {}, {}".format(cx, cy))

                # Draw a circle at the centroid coordinates
                # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                cv2.circle(hsv_image, (round(cx), round(cy)), 5,(0, 100, 100), -1)

                # if target is to the left turn left
                if cx < data.width / 3:
                    self.tw.angular.z=0.3
                # if target is to the right turn right
                elif cx >= 2 * data.width / 3:
                    self.tw.angular.z=-0.3
                else: 
                    #centre of object is within a 100px range of the centre of the image
                    self.tw.angular.z=-0.0
                    print("en route")    
        else:
            print("No target fount")
            self.tw.angular.z=0.3 # Turn left until we find something
        
        self.pub_cmd_vel.publish(self.tw)

        #img_resized = cv2.resize(image_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        #cv2.imshow("Image window", current_frame_contours_small)
        #cv2.waitKey(1)


def main(args=None):
    print('Starting colour_chaser.py')
    cv2.startWindowThread()

    rclpy.init(args=args)

    colour_chaser = ColourChaser()

    rclpy.spin(colour_chaser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_chaser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
