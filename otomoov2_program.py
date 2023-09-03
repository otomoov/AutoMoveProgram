#! /usr/bin/env python3

# Otomoov2 program 0.5

import time
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
import pyfirmata

from robot_navigator import BasicNavigator, NavigationResult

class Otomoov():

    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()
        self.goal_pose = PoseStamped()
        self.ctrl_c = False
        self.board = pyfirmata.Arduino('/dev/ttyUSB1')
        self.it = pyfirmata.util.Iterator(self.board)
        self.it.start()
        self.board.digital[10].mode = pyfirmata.INPUT

    '''Basic navigation demo to go to pose.'''

    def goToGoal(self):
        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        # Go to our demos first goal pose
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 4.5
        self.goal_pose.pose.position.y = 1.5
        self.goal_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(self.goal_pose)

        print('Sleepng for 10sec\n')
        time.sleep(10)

    def goToGoal1(self):
        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        # Go to our demos first goal pose
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 1.0
        self.goal_pose.pose.position.y = -2.0
        self.goal_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(self.goal_pose)

        print('Sleepng for 10sec\n')
        time.sleep(10)
 

    def goToHome(self):
        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        # Go to our demos first goal pose
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 0.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.orientation.w = 1.0

        self.navigator.goToPose(self.goal_pose)

    def selectKeyboard(self):
        keyInput = input("Press 1 the press ENTER to send otomoov to goal:\n")
        if keyInput == '1':
            print('key input "1" received')
            print('moving to goal\n')
            self.goToGoal()
            print('Goal Reached\n')
            self.sleeping()
            print('Moving to Home\n')
            self.goToHome()
            print('Home Reached\n')
        else:
            print('wrong key received. exiting...\n')
            print('exited')
            exit(0)

    def selectButton(self):
        pushButtonGreen = self.board.digital[10].read()
        print('waiting for Button')
        if pushButtonGreen is True:
            self.board.digital[13].write(1)
            print('Button pushed, LED ON')
            print('Moov (Green) button pushed')
            print('moving to goal\n')
            self.goToGoal()
            #self.goToGoal1()
            print('Goal Reached\n')
            self.sleeping()
            print('Moving to Home\n')
            #self.goToGoal()
            self.goToHome()
            print('Home Reached\n')
        else:
            self.board.digital[13].write(0)
            print('Button release, LED OFF')
        time.sleep(1)


    def sleeping(self):
        print('sleeping for 10 seconds...')
        time.sleep(2)
        print('slept for 2sec')
        time.sleep(2)
        print('slept for 4sec')
        time.sleep(2)
        print('slept for 6sec')
        time.sleep(2)
        print('slept for 8sec')
        time.sleep(2)
        print('slept for 10sec')
        print('done sleeping. going back to work\n')

oto = Otomoov()

if __name__ == '__main__':
    while True:
        # oto.selectKeyboard()
        oto.selectButton()
