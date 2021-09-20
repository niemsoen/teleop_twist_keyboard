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
r   : reset speeds to default

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


def printClean(settings, msg):
    # change terminal settings temporarily for standard printing
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(msg)
    # default state: setup terminal for getKey
    tty.setraw(sys.stdin.fileno())


def getKey(settings, keyLock, keyPublisher):
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
            key = sys.stdin.read(1) # this is blocking
            keyChanged = True                        

        if keyChanged:
            keyLock.acquire(blocking=True)
            lastkey = key
            keyLock.release()

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


def paramsToStr(node):
    p = (node.targetSpeed, node.targetTurn, node.accel)
    return 'speeds:\tlinear %.2f\tturn %.2f\tmax_accel: %.2f' % p


def evaluateKey(keyLock, node):
    global lastkey
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    printMsgEvery = 6
    lastCmdWasHalt = False
    keyWasPressed = True
    consecutiveMoveSteps = 0
    
    while True:
        # print help message every X lines
        if (status == printMsgEvery):
            printClean(node.termSet, msg)
            printClean(node.termSet, paramsToStr(node))
            status = (status + 1) % (printMsgEvery+1)

        # make copy of key that is evaluated
        key_copy = lastkey
        if key_copy != '':
            keyWasPressed = True
            keyLock.acquire(blocking=True)
            lastkey = ''
            keyLock.release()
        
        if key_copy in moveBindings.keys():
            if lastCmdWasHalt:
                printClean(node.termSet, 'moving...')
                status = (status + 1) % (printMsgEvery+1)
                consecutiveMoveSteps = 0
            lastCmdWasHalt = False
            # accelerate the robot
            consecutiveMoveSteps += 1
            timeSinceHalt = consecutiveMoveSteps * readKeyTimeout
            node.speed = timeSinceHalt * node.accel
            node.turn = timeSinceHalt * node.accel
            if node.speed >= node.targetSpeed:
                node.speed = node.targetSpeed
            if node.turn >= node.targetTurn:
                node.turn = node.targetTurn
            
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
        elif key_copy in speedBindings.keys():
            node.targetSpeed = node.targetSpeed * speedBindings[key_copy][0]
            node.targetTurn = node.targetTurn * speedBindings[key_copy][1]
            node.updateRosParams()
            
            # send halt command
            node.sendHalt()
            lastCmdWasHalt = True

            # print updated speeds
            printClean(node.termSet, paramsToStr(node))
            
            # increment status for help message printing
            status = (status + 1) % (printMsgEvery+1)
        else:
            # check for ctrl+c
            if (key_copy == '\x03'):                
                break

            # reset speeds to default
            if (key_copy == 'r'):
                node.targetSpeed = 0.25
                node.targetTurn = 0.25
                printClean(node.termSet, "Reset speeds:")
                node.updateRosParams()
                printClean(node.termSet, paramsToStr(node))
                # increment status for help message printing
                status = (status + 1) % (printMsgEvery+1)
            
            # send halt command, avoiding repetition
            if not lastCmdWasHalt:
                node.sendHalt()
                lastCmdWasHalt = True
                printClean(node.termSet, 'STOP (Key released)')
                status = (status + 1) % (printMsgEvery+1)                

        if keyWasPressed:
            time.sleep(readKeyTimeout)


class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')

        self.declare_parameter('speed', 0.25)
        self.declare_parameter('turn', 0.25)
        self.declare_parameter('max_accel', 0.25)
        
        self.targetSpeed = self.get_parameter('speed').get_parameter_value().double_value
        self.targetTurn = self.get_parameter('turn').get_parameter_value().double_value
        self.accel = self.get_parameter('max_accel').get_parameter_value().double_value
        
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def parameters_callback(self, params):
        for param in params:
            if param.name == "speed":
                self.targetSpeed = param.value
            if param.name == "turn":
                self.targetTurn = param.value
            if param.name == "max_accel":
                self.accel = param.value
        return SetParametersResult(successful=True)

    ''' Updates the ROS-parameters after the local speed and turn are changed
    '''
    def updateRosParams(self):
        param_speed = Parameter('speed', Parameter.Type.DOUBLE, self.targetSpeed)
        param_turn = Parameter('turn', Parameter.Type.DOUBLE, self.targetTurn)
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
    keyLock = threading.Lock()

    # print help message
    print(msg)
    print(paramsToStr(node))

    # thread for evaluating the key that was read
    evalThread = threading.Thread(target=evaluateKey, args=(keyLock,node,), name='evalThread')
    keyThread = threading.Thread(target=getKey, args=(node.termSet, keyLock, node.pubKey), name='getKeyThread')
    
    try:
        evalThread.start() 
        keyThread.start()
        rclpy.spin(node)

    except Exception as e:
        node.get_logger().info('Exception occured in executing threads: ' + str(e))
    
    finally:
        evalThread.join()
        
        node.sendHalt()
        node.get_logger().info('Sent STOP command, shutting down')
        
        restoreTerminalSettings(node.termSet)        


if __name__ == '__main__':
    main()
