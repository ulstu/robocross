#!/usr/bin/env python3

import rospy
import time
import roslib
import math
import sys
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from simple_pid import PID
#from cv_bridge import CvBridge, CvBridgeError


class PathDetector:
    def __init__(self):
        self.is_init = False
        self.angle_pub = rospy.Publisher("serialcode", String, queue_size=1)
#        self.angle_pub = rospy.Publisher("angle", std_msgs.msg.String, queue_size=5)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)

        time.sleep(10)
        self.image_sub = rospy.Subscriber("/img_node/range_image", Image, self.image_callback)
        self.carstate_sub = rospy.Subscriber("carstate", String, self.carstate_callback)
        self.is_turn = False
        self.is_stop = False
        self.pos_history = []
        self.pid = PID(1, 0.1, 0.05, setpoint=20)
        self.pid_barrel = PID(1, 0.1, 0.05, setpoint=200)
        self.pid.sample_time = 0
        self.pid_barrel.sample_time = 0
        self.barrel = False
        self.is_viz = True

        self.is_init = True

    def carstate_callback(self, data):
        if 'turn' in data.data:
            self.is_turn = True
        if 'stop' in data.data:
            self.is_stop = True
    
    def detect_blocks(self, image):
        contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        min_len2center = 90000
        needed_block = []
        for contour in contours:
            box = cv2.boundingRect(contour)
            x, y, w, h = box
            len1 = abs(x - 500)
            len2 = abs(x + w - 500)
            if len1 + len2 < min_len2center:
                needed_block = box
                min_len2center = len1 + len2
        # m = np.median(img)
        #print(m)
        # self.angle_pub.publish("6 " + str(m))
        return needed_block
    
    def new_detect_blocks(self, img):
        
        return
    
    def search_window(self, np_img, img_height):

        (rows, cols, channels) = np_img.shape
        window_size = 190 #190
        imin = 0
        stride = 1
        minw = (255 * img_height * window_size)
        for i in range(0, np_img.shape[1] - window_size, stride):
            window = np_img[:, i : i + window_size,:]
            s = np.sum(window)
            #print(s, minw)
            #if abs(minw - s) < 10000 or s < minw or (abs(imin - i) < 10 and abs(s - minw) < 1000):
            if s < minw + 1000:# or  (abs(imin - i) < 10 and abs(s - minw) < 1000):
                minw = s
                imin = i
        return minw, imin

    def check_barrel(self, image):

        limit = 500
        contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        min_len2center = 10000
        needed_block = []
        rightest_block = []
        leftest_block = []
        min_len2right = 0
        min_len2left = 10000
        for contour in contours:
            box = cv2.boundingRect(contour)
            x, y, w, h = box
            len1 = abs(x - 500)
            len2 = abs(x + w - 500)
            if len1 + len2 < min_len2center and w > 40 and np.median(image[2, x:x+w, :]) > 170 and x > 10 and x+w < 990:
                needed_block = box
                min_len2center = len1 + len2
            if w > 200 and x+w > min_len2right:
                rightest_block = box
                min_len2right = x+w
            if w > 200 and x < min_len2left:
                leftest_block = box
                min_len2left = x
            print(box)
        if needed_block and limit < needed_block[0]+needed_block[2]:
            self.barrel = True
        return needed_block
        # if self.is_viz:
        #     image = cv2.resize(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), (image.shape[1], 300))



    def image_callback(self, data):
        if self.is_init:
            try:
                cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
                #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                #rospy.loginfo(f'image received {cv_image.shape}')
            except Exception as e:
                print(e)

            needed_block = []
            padding = 500
            right_wall = 255 - np.median(cv_image[8, 1365:1707, :])
            analyzed_image = cv_image[6:9, padding:-padding, :]

            check_angle_wall = min(60, max(30, self.pid(right_wall)))
            if self.barrel:
                x, y, w, h = self.detect_blocks(analyzed_image)
                range = 255 - np.median(analyzed_image[2, x:x+w, :])
                if 40 < w < 350 and range < 85:
                    angle = min(90, max(-90, self.pid(x-500)))
                    print(f'angle barrel = {angle}')
                else:
                    self.barrel = False

            else:
                needed_block = self.check_barrel(analyzed_image)
                angle = min(60, max(-60, self.pid(right_wall)))

            if self.is_viz:
                center = cv_image.shape[1] // 2
                viz_image = cv2.resize(cv2.cvtColor(cv_image[6:9, padding:-padding, :], cv2.COLOR_GRAY2BGR), (cv_image.shape[1], 300))
                if self.barrel:
                    cv2.rectangle(viz_image, (x, y*100), (x+w, (y+h)*100), (255, 0, 0), 2)
                    x = int(400 * math.sin(angle) * 180 / math.pi)
                    y = int(100 * math.cos(angle) * 180 / math.pi)
                    cv2.line(viz_image, (angle, 299), (center + x, 299 - y), (0, 0, 255), 2)


                x = int(400*math.sin(check_angle_wall) * 180 / math.pi)
                y = int(100 * math.cos(check_angle_wall) * 180 / math.pi)
                cv2.line(viz_image, (check_angle_wall, 299), (center+x, 299 - y), (0, 255, 0), 2)
                cv2.imshow("Image window", viz_image)
                cv2.waitKey(3)

            if self.is_turn:
                x, y, w, h = self.detect_blocks(analyzed_image)
                block_distance = np.median(analyzed_image[2, x:x+w, :])
                left_edge = np.median(cv_image[6:9, 341:682, :])
                right_edge = np.median(cv_image[6:9, 1365:1707, :])
                print(left_edge, right_edge)
                if right_edge > left_edge and block_distance > 170:
                    self.angle_pub.publish("5 1")
                    self.is_turn = False
                elif right_edge < left_edge and block_distance > 190:
                    self.angle_pub.publish("5 2")
                    self.is_turn = False
            if self.is_stop:
                x, y, w, h = self.detect_blocks(analyzed_image)
                block_distance = np.median(analyzed_image[2, x:x + w, :])
                if block_distance > 145:
                    self.is_stop = False
                    self.angle_pub.publish("7 0")




            # #rospy.loginfo("image shape {} {} {}".format(rows, cols, channels))
            # scale_percent = 30  # percent of original size
            # width = int(cv_image.shape[1] * scale_percent / 100)
            # height = int(cv_image.shape[0] * scale_percent / 100)
            # dim = (width, height)
            # # resize image
            # #cv_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)
            #
            #
            # np_img = cv_image[6:9, padding:-padding,:].copy()
            # np_img[np_img < 150] = 0
            # block_distance = self.detect_blocks(np_img)
            # #block_distance = np.median(cv_image[6, 1100:1200, :])
            # #print(block_distance) #145 140
            # left_edge = np.median(cv_image[6:9, 341:682, :])
            # right_edge = np.median(cv_image[6:9, 1365:1707, :])
            # #print(left_edge, right_edge)
            #
            #
            # img_height = 66
            # new_img = np.empty((img_height, np_img.shape[1], np_img.shape[2]), dtype=np.uint8)
            # for i in range(img_height):
            #     r = int(i / int(img_height / np_img.shape[0]))
            #     new_img[i, :, :] = np_img[r, :, :]
            # np_img = new_img
            #
            # (rows, cols, channels) = np_img.shape
            # window_size = 190
            # stride = 1
            # #rear = np_img.copy()
            # #rear[rear < 230] = 0
            #
            # minw, imin = self.search_window(np_img, img_height)
            # #rear_minw, rear_imin = self.search_window(rear, img_height)
            #
            # if (imin + window_size / 2 > int(cols / 2)):
            #     imin -= 50
            #
            # np_img = cv2.cvtColor(np_img.astype(np.uint8),cv2.COLOR_GRAY2RGB)
            #
            # cv2.rectangle(np_img, (imin, 0), (imin + window_size, rows), (0, 255, 0), 2)
            # cv2.line(np_img, (imin + int(window_size / 2), 0), (int(cols / 2), img_height), (255, 0, 0), 3)
            # #cv2.line(np_img, (rear_imin + int(window_size / 2), 0), (int(cols / 2), img_height), (0, 0, 255), 3)
            # angle = int(np.arctan(((imin + int(window_size / 2) - cols / 2) / img_height))  * 90)
            #
            # angle =  angle ** 2 / 75 * (-1 if angle < 0 else 1)
            # #print(f'angle: {angle}')
            # #if (angle > 0):
            # #    angle = angle - 15
            # #else:
            # #    angle = angle + 15
            # self.angle_pub.publish("16 " + str(angle * 12))


            try:
                # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                pass
            except CvBridgeError as e:
                print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('image_obstacle_detection', anonymous=True)
        detector = PathDetector()
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка car node: {}'.format(traceback.format_exc()))
