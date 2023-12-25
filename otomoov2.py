#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
# Copyright 2023 Otomoov Technology
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

# Otomoov2 program 0.9.2

import pyfirmata
import rclpy
import time
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String

"""
Button to command Linorobot2 to go to goal. 
"""

class Otomoov():

    def __init__(self):
        rclpy.init()
        
        self.goal_pose = PoseStamped()
        self.navigator = BasicNavigator()
        self.ctrl_c = False

        self.TwistMsg = geometry_msgs.msg.Twist
        self.node = rclpy.create_node('teleop_twist_keyboard')
        self.pub = self.node.create_publisher(self.TwistMsg, 'cmd_vel', 10)
        self.twist_msg = self.TwistMsg()
        self.twist = self.twist_msg

        #self.board = pyfirmata.Arduino('/dev/ttyUSB1')
        self.board = pyfirmata.Arduino('/dev/ttyACM1')
        self.it = pyfirmata.util.Iterator(self.board)
        self.it.start()
        self.board.digital[2].mode = pyfirmata.INPUT
        self.board.digital[3].mode = pyfirmata.INPUT
        self.board.digital[5].mode = pyfirmata.INPUT
        self.board.digital[6].mode = pyfirmata.INPUT

    """ Go To """

    def goToGoal(self):
        self.setPose(4.0, -0.5)

    def goToGoal2(self):
        self.setPose(3.0, 0.5)

    def goToHome(self):
        self.setPose(0.0, 0.0)

    """ Set Pose """
    
    def setPose(self, poseX, poseY):
        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        # Go to goal pose
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = poseX
        self.goal_pose.pose.position.y = poseY
        self.goal_pose.pose.orientation.w = 1.0
        self.navigator.goToPose(self.goal_pose)

    def setHome(self):
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)
        print('Otomoov Home\n')

    """ Blink """

    def blinkPrint(self, ledPin):
        if ledPin == 13:
            print('blinking Led Green')
        else:
            print('blinking Led Yellow')

    def blinkOnce(self, ledPin1, ledPin2, switch1, switch2, switch3, switch4):
        self.board.digital[ledPin1].write(switch1)
        self.board.digital[ledPin2].write(switch2)
        time.sleep(0.5)
        self.board.digital[ledPin1].write(switch3)
        self.board.digital[ledPin2].write(switch4)
        time.sleep(0.5)

    def blinkTimes(self, times, ledPin1, ledPin2, switch1, switch2, switch3, switch4):
        for x in range(0,times):
            self.blinkOnce(ledPin1, ledPin2, switch1, switch2, switch3, switch4)

    """ Docking """

    def dockSearch(self):
        self.for_rev = "reverse"
        while not self.board.digital[5].read() !=1:
            time.sleep(0.01)
            self.move(self.for_rev, 0.0, 0.1)

    def predock(self):
        print("otomoov moving to pre-dock coordinates...\n")
        self.setPose(0.2, 0.0)

    def move(self, direction, speedLinear, speedAngular):
        # Initilize velocities
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = speedAngular

        if direction == "forward":
            self.twist.linear.x = speedLinear     #"forward" dock
            #print('Forward Docking...\n')
        else:
            self.twist.linear.x = -speedLinear    #"reverse" dock
            #print('Reverse Docking...\n')
		# Publish the velocity
        self.pub.publish(self.twist_msg)

    def stopRobot(self):
        #self.get_logger().info("STOPPING....")
        print('Stopping Robot...\n')
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist_msg)
        #self.get_logger().info("STOPPED")

    def dock(self):
        #print('Docking Operation Started...\n')
        self.revDock = "reverse"
        while True:
            #print('Docking...\n')
            if (self.board.digital[5].read() and self.board.digital[6].read()) != 0:
                print('move')
                time.sleep(0.01)
                self.move(self.revDock, 0.05, 0.0)
            if self.board.digital[5].read() !=1:
                print('line sensor rear left ON!')
                time.sleep(0.01)			
                self.move(self.revDock, 0.01, -0.1) # rotate clockwise
            if self.board.digital[6].read() !=1:
                print('line sensor rear right ON!')
                time.sleep(0.01)
                self.move(self.revDock, 0.01, 0.1) # rotate counter-clockwise
            if (self.board.digital[5].read() != 1) and (self.board.digital[6].read() != 1):
                print('stop\n')
                self.stopRobot()
                break

    """ Wait for Nav2 to complete task """

    def waitTask(self):
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()
            
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    """ Selection Keyboard """
    
    def selectKeyboard(self):
        keyInput = input("Press 1 the press ENTER to send otomoov to goal:\n")
        if keyInput == '1':
            print('key input "1" received')
            print('moving to goal\n')
            self.goToGoal()
        else:
            print('wrong key received. exiting...\n')
            print('exited')
            exit(0)

    """ Selection Button """

    def selectButton(self):
        pushButtonG = self.board.digital[2].read() #pin D3
        pushButtonY = self.board.digital[3].read() #pin D4
        result = self.navigator.getResult()
        if pushButtonG is True:
            self.board.digital[12].write(0)
            self.board.digital[13].write(1)
            self.setHome()
            print('Button pushed, LED Green ON')
            print('Moov (Green) button pushed')
            print('moving to goal 1\n')
            self.goToGoal()
            self.waitTask()
            self.blinkTimes(5, 12, 13, 1, 1, 0, 0)
            self.predock()
            self.waitTask()
            self.blinkTimes(5, 12, 13, 1, 1, 0, 0)
            self.dock()
            self.setHome()
            print('task completed')
            print('Otomoov Waiting for Button Push, LED OFF')

        if pushButtonY is True:
            self.board.digital[12].write(1)
            self.board.digital[13].write(0)
            self.setHome()
            print('Button pushed, LED Yellow ON')
            print('Moov (Yellow) button pushed')
            print('moving to goal 2\n')
            self.goToGoal()
            self.waitTask()
            self.blinkTimes(5, 12, 13, 1, 1, 0, 0)
            self.predock()
            self.waitTask()
            self.blinkTimes(5, 12, 13, 1, 1, 0, 0)
            self.dock()
            self.setHome()
            print('task completed')
            print('Otomoov Waiting for Button Push, LED OFF')

        else:				
            self.blinkOnce(12, 13, 1, 1, 0, 0)
        time.sleep(0.1)

if __name__ == '__main__':
    oto = Otomoov()
    print('Otomoov program running...\n')
    print('Otomoov Waiting for Button Push, LED OFF')
    while not oto.ctrl_c:
        oto.selectButton()
