# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

import geometry_msgs.msg
import rclpy

import rclpy.node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
import threading
import time

from std_msgs.msg import String


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

lastkey = ''
readKeyTimeout = 0.1

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def printClean(lock, settings, msg):
    lock.acquire(blocking = True)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(msg)
    lock.release()


def getKey(settings, lock, keyPublisher):
    global lastkey
    keyChanged = False
    key = '' # local read variable for key
    while True:        
        if sys.platform == 'win32':
            # getwch() returns a string on Windows    
            key = msvcrt.getwch()
            keyChanged = True
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            key = sys.stdin.read(1)
            keyChanged = True
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)        

        if keyChanged:
            lock.acquire(blocking=True)
            lastkey = key
            lock.release()

            msg = String()
            msg.data = 'last key pressed:' + lastkey
            keyPublisher.publish(msg)
            keyChanged = False            

            # check for ctrl+c
            if (key == '\x03'):
                break


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(_speed, _turn):
    return 'currently:\tspeed %s\tturn %s ' % (_speed, _turn)


def evaluateKey(lock,node):
    global lastkey
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    printMsgEvery = 10
    lastCmdWasHalt = False
    keyWasPressed = True
    
    while True:
        # make copy of key that is evaluated
        key_copy = lastkey
        if key_copy != '':
            keyWasPressed = True
            lock.acquire(blocking=True)
            lastkey = ''
            lock.release()
        
        if key_copy in moveBindings.keys():
            x = moveBindings[key_copy][0]
            y = moveBindings[key_copy][1]
            z = moveBindings[key_copy][2]
            th = moveBindings[key_copy][3]
            
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * node.speed
            twist.linear.y = y * node.speed
            twist.linear.z = z * node.speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * node.turn
            
            node.pubTwist.publish(twist)
            lastCmdWasHalt = False
        
        elif key_copy in speedBindings.keys():
            node.speed = node.speed * speedBindings[key_copy][0]
            node.turn = node.turn * speedBindings[key_copy][1]
            node.updateRosParams()
            
            # send halt command
            node.sendHalt()
            lastCmdWasHalt = True

            #print(vels(node.speed, node.turn))
            msg = vels(node.speed, node.turn)
            printClean(lock, node.termSet, msg)
            # increment status for help message printing
            status = (status + 1) % (printMsgEvery+1)
        
        else:
            # check for ctrl+c
            if (key_copy == '\x03'):                
                break
            
            # send halt command, avoiding repetition
            if not lastCmdWasHalt:
                node.sendHalt()
                lastCmdWasHalt = True
                print('no move/speed cmd: sent STOP')
                status = (status + 1) % (printMsgEvery+1)
        
        # print help message every X lines
        if (status == printMsgEvery):
            print(msg)
            print(vels(node.speed, node.turn))
            status = (status + 1) % (printMsgEvery+1)

        if keyWasPressed:
            time.sleep(readKeyTimeout)


class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard_rosrun')

        self.declare_parameter('speed', 1.0)
        self.declare_parameter('turn', 1.0)
        #self.declare_parameter('readKeyTimeout', 0.1)
        
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.turn = self.get_parameter('turn').get_parameter_value().double_value
        #readKeyTimeout = self.get_parameter('readKeyTimeout').get_parameter_value().double_value
        
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def parameters_callback(self, params):
        for param in params:
            if param.name == "speed":
                self.speed = param.value
            if param.name == "turn":
                self.turn = param.value
        #print(vels(self.speed, self.turn))
        return SetParametersResult(successful=True)

    ''' Updates the ROS-parameters after the local speed and turn are changed
    '''
    def updateRosParams(self):
        param_speed = Parameter('speed', Parameter.Type.DOUBLE, self.speed)
        param_turn = Parameter('turn', Parameter.Type.DOUBLE, self.turn)
        self.set_parameters([param_speed, param_turn])
    
    ''' Sends halt command on the topic /cmd_vel
    '''
    def sendHalt(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        self.pubTwist.publish(twist)


def main():
    rclpy.init()
    
    node = MinimalParam()
    
    node.get_logger().info('Started teleop')

    node.termSet = saveTerminalSettings()
    node.pubTwist = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
    node.pubKey = node.create_publisher(String, 'key_pressed', 10)

    # lock for the 'lastkey' variable
    lock = threading.Lock()

    # print help message
    print(msg)
    print(vels(node.speed, node.turn))

    # thread for evaluating the key that was read
    evalThread = threading.Thread(target=evaluateKey, args=(lock,node,), name='evalThread')
    
    try:
        # launches evaluating key reading in seperate thread
        evalThread.start() 

        # launches loop for key reading in main thread
        getKey(node.termSet, lock, node.pubKey) # blocking

    except Exception as e:
        node.get_logger().info('Exception occured in executing threads: ' + str(e))
    
    finally:
        evalThread.join()
        node.sendHalt()
        node.get_logger().info('Sent STOP command, shutting down')
    
    restoreTerminalSettings(termSet)


if __name__ == '__main__':
    main()
