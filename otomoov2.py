#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
# Copyright 2023 Azareka Enterprise
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

# Otomoov2 program 0.5

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import rclpy
import time
import pyfirmata

"""
Button to command Linorobot2 to go to goal. 
"""

class Otomoov():

    def __init__(self):
        rclpy.init()
        
        self.navigator = BasicNavigator()
        self.goal_pose = PoseStamped()
        self.ctrl_c = False
        self.board = pyfirmata.Arduino('/dev/ttyUSB1')
        self.it = pyfirmata.util.Iterator(self.board)
        self.it.start()
        self.board.digital[2].mode = pyfirmata.INPUT
        self.board.digital[3].mode = pyfirmata.INPUT
        self.board.digital[4].mode = pyfirmata.INPUT

    """ Go to goal """

    def goToGoal(self):
        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        # Go to our demos first goal pose
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 1.0
        self.goal_pose.pose.position.y = -3.0
        self.goal_pose.pose.orientation.w = 1.0
        self.navigator.goToPose(self.goal_pose)

        print('Sleepng for 10sec\n')
        oto.sleeping()
        #time.sleep(10)
        
    """ Go to goal2 """

    def goToGoal2(self):
        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        # Go to our demos first goal pose
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 1.5
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.orientation.w = 1.0
        self.navigator.goToPose(self.goal_pose)

        print('Sleepng for 10sec\n')
        oto.sleeping()
        #time.sleep(10)

    """ Go to home """
    
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

    """ Selection Keyboard """
    
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

    """ Selection Button """
    
    def selectButton(self):
        pushButton2 = self.board.digital[3].read() #pin D3
        pushButton3 = self.board.digital[4].read() #pin D4
        self.blinkG()
        print('waiting for Button')
        if pushButton3 is True:
            self.board.digital[13].write(1)
            print('Button pushed, LED ON')
            print('Moov (Green) button pushed')
            print('moving to goal\n')
            self.goToGoal()
            print('Goal Reached\n')
            self.blinkYG()
            self.sleeping()
            print('Moving to Home\n')
            self.goToHome()
            print('Home Reached\n')
            self.blinkRYG()
        if pushButton2 is True:
            self.board.digital[13].write(1)
            self.board.digital[12].write(1)
            print('Button pushed, LED ON')
            print('Moov (Green) button pushed')
            print('moving to goal\n')
            self.goToGoal2()
            print('Goal Reached\n')
            self.blinkYG()
            self.sleeping()
            print('Moving to Home\n')
            self.goToHome()
            print('Home Reached\n')
            self.blinkRYG()
        else:
            self.board.digital[13].write(0)
            print('Button release, LED OFF')
        time.sleep(1)

    """ Blink """
    
    def sleeping(self):
        self.blinkG()
        print('sleeping for 10 seconds...')
        self.blinkG()
        print('slept for 2sec')
        self.blinkG()
        print('slept for 4sec')
        self.blinkG()
        print('slept for 6sec')
        self.blinkG()
        print('slept for 8sec')
        self.blinkG()
        print('slept for 10sec')
        print('done sleeping. going back to work\n')
        
    """ Blink """
    
    def blinkG(self):
        print('blinkGreen...')
        self.board.digital[13].write(1)
        time.sleep(1)
        self.board.digital[13].write(0)
        time.sleep(1)
        
    def blinkRYG(self):
        print('blinkRYG...')
        self.board.digital[11].write(1)
        self.board.digital[12].write(1)
        self.board.digital[13].write(1)
        time.sleep(1)
        self.board.digital[11].write(0)
        self.board.digital[12].write(0)
        self.board.digital[13].write(0)
        time.sleep(1)

    def blinkYG(self):
        print('blinkRYG...')
        self.board.digital[12].write(1)
        self.board.digital[13].write(1)
        time.sleep(1)
        self.board.digital[12].write(0)
        self.board.digital[13].write(0)
        time.sleep(1)

oto = Otomoov()

if __name__ == '__main__':
    while True:
        # oto.selectKeyboard()
        oto.selectButton()
