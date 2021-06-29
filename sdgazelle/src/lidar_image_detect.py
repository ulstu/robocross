#!/usr/bin/env python3

import rospy
import roslib
import sys
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError


class ObstacleDetector:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)
        #self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/img_node/intensity_image", Image, self.image_callback)
        self.pos_history = []

    def image_callback(self, data):
        try:
            cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #rospy.loginfo(f'image received {cv_image.shape}')
        except Exception as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        scale_percent = 30  # percent of original size
        width = int(cv_image.shape[1] * scale_percent / 100)
        height = int(cv_image.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        #cv_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

        window_size = 128
       
        min = 255 * 64 * window_size
        imin = 0
        stride = 10
        padding = 500
        for i in range(padding, cv_image.shape[1] - window_size - padding, stride):
            window = cv_image[:,i : i + window_size,:]
            s = np.sum(window)
            if s <= min:
                min = s
                imin = i
        if len(self.pos_history) > 0 and abs(imin - self.pos_history[-1]) > 20:
            imin = self.pos_history[-1]
        filter_size = 3
        # if len(self.pos_history) > filter_size:
        #     imin = int((np.mean(self.pos_history[:-filter_size:-1]) + imin) / 2)
        self.pos_history.append(imin)
        cv2.rectangle(cv_image, (imin, 0), (imin + window_size, rows), (0, 255, 0), 2)
        cv2.line(cv_image, (imin + 64, 0), (int(cols / 2), rows), (255, 0, 0), 4)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            pass
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('image_obstacle_detection', anonymous=True)
        detector = ObstacleDetector()
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка car node: {}'.format(traceback.format_exc()))
