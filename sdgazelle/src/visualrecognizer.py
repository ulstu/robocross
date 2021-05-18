#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from utils import *
import traceback
import time


class VisualRecognizer(object):
    control_pub = None
    cmd_pub = None

    def publish_control(self, velocity, wheel, transmission):
        msg = ""
        if velocity != None:
            msg = "velocity: {};".format(velocity)
        if wheel != None:
            msg += "wheel: {};".format(wheel)
        if transmission != None:
            msg += "transmission: {}".format(transmission)
        self.control_pub.publish(msg)

    def start_car(self):
        '''
        Начало движения
        '''
        log(self, "start")
        self.publish_control(5, 0, 1)
        # запуск распознавания
    
    def stop_car(self):
        '''
        Остановка алгоритма распознаания
        '''
        log(self, "stop")
        self.publish_control(0, 0, 0)

    def turn_car(self):
        '''
        Разворот автомобиля
        '''
        log(self, "turn")
        self.publish_control(5, 600, -1)
        rospy.sleep(2)
        self.publish_control(5, -600, 1)
        rospy.sleep(2)
        self.publish_control(5, 600, -1)
        rospy.sleep(2)
        self.publish_control(5, -600, 1)
        rospy.sleep(2)
        self.publish_control(5, 0, 1)
        rospy.sleep(2)
        self.cmd_pub.publish("finishturn")

    def finish_car(self):
        '''
        Завершение выполнения задания
        '''
        log(self, "finish")
        self.publish_control(0, 0, 0)

    def cmd_callback(self, data):
        '''
        Callback для получения сигнала управления автомобилем
        '''
        log(self, "data received: {}".format(data))
        data = str(data).replace("data:", "")
        vars = str(data).split(';')
        cmd = {}
        for v in vars:
            keyval = v.split(':')
            cmd[clear_str(keyval[0])] = clear_str(keyval[1])
        self.cur_cmd = {}
        if "cmd" in cmd:
            if cmd["cmd"] == "start":
                self.start_car()
            elif cmd["cmd"] == "stop":
                self.stop_car()
            elif cmd["cmd"] == "turn":
                self.turn_car()

    def start(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.2)

    def __init__(self):
        rospy.Subscriber("carcmd", String, self.cmd_callback)
        self.control_pub = rospy.Publisher('carcontrol', String, queue_size=10)
        self.cmd_pub = rospy.Publisher('completetask', String, queue_size=10)

if __name__ == '__main__':
    try:
        rospy.init_node('visualrecognizer', anonymous=True)
        ctrl = VisualRecognizer()
        ctrl.start()
    except rospy.ROSInterruptException:
        rospy.logerr('Ошибка car node: {}'.format(traceback.format_exc()))
