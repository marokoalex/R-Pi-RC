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
import re
from rclpy.node import Node

from std_msgs.msg import String

 
import time
from adafruit_motorkit import MotorKit

import math


class RCCmdSubscriber(Node):

    def __init__(self):
        super().__init__('rccmd_subscriber')
        self.subscription = self.create_subscription(
            String,
            'rc_cmd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.motorKit = MotorKit(64) # 0x40 the i2c address
        self.power = 0
        self.angle = 0

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        ##https://stackoverflow.com/questions/58033614/python-regex-to-extract-positive-and-negative-numbers-between-two-special-charac
        cmds = re.findall(r'[-+]?(?:\d{1,3}(?:,\d{3})+|\d+)(?:\.\d+)?', msg.data)
        if len(cmds) == 2:
            print("angle/power: " + str(cmds))
            self.angle = int(str(cmds[0]))
            print("angle: " + str(cmds[0]))
            self.power = int(str(cmds[1]))
            print("power: " + str(cmds[1]))
        else:
            print("error cmd format")
        
        #s_outer = k*s_inner
        #k=114.59/(114.59-angle_deg)
        #k_max could be ~5.0 for 2 90 deg turn with radius of ~2w 
        #(w the dist between inner and outer wheels)
        #so we make the max power of motor to be 0.2
        #for the outer mortor to be able to reach ~5 times of s_inner
        
        s_inner = self.power*0.2
        k = 114.59/(114.59-self.angle)
        s_outer = k*s_inner
        
        self.motorKit.motor1 = s_inner
        self.motorKit.motor3 = s_inner
        
        self.motorKit.motor2 = s_outer
        self.motorKit.motor4 = s_outer                


def main(args=None):
    s = "You are...my fire...the one...desire"
    # split string by substring "..."
    ls = s.split("...")
    print(ls)


    
    rclpy.init(args=args)

    cmd_subscriber = RCCmdSubscriber()

    rclpy.spin(cmd_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
