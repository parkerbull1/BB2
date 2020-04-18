#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import numpy as np
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import Int8

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):        
	self.image_pub = rospy.Publisher("Camera", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.Callback)
	self.bridge = CvBridge()

    def Callback(self,data):
        try: 
	    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
	    print(e)

        red_low = np.array([0, 0, 0])
	red_high = np.array([15, 15, 255])
	resized = cv2.resize(cv_image, (25, 25), interpolation = cv2.INTER_AREA)

	red_mask = cv2.inRange(resized, red_low, red_high)
	red_out = cv2.bitwise_and(resized, resized, red_mask, red_mask)
	redout = cv2.resize(red_out, (800, 800), interpolation = cv2.INTER_AREA)

        
	
	try:
	    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
	except CvBridgeError as e:
	    print(e)

	

	cv2.imshow("Camera Feed", cv_image)
	cv2.imshow("Red Mask", redout)
	#print(cv_image[300, 440])
	cv2.waitKey(3)

	

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    try:
	rospy.spin()
    except KeyboardInterrupt:
	print("Shutting Down")
    cv2.destroyAllWindows()



if __name__ == '__main__':
  main(sys.argv)

