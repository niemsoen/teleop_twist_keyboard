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
import threading
import time

from std_msgs.msg import String

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty



class TeleopKeyboard(rclpy.node.Node):
    
    def __init__(self):
        super().__init__('teleop_twist_keyboard')

        self.declare_parameter('target_speed', 0.25) # in m/s
        self.declare_parameter('target_turn', 0.25)  # in m/s
        self.declare_parameter('max_accel', 0.25)  # in m/s^2
        self.declare_parameter('max_deccel', -5.0) # in m/s^2
        
        self.target_speed = self.get_parameter('target_speed').get_parameter_value().double_value
        self.target_turn = self.get_parameter('target_turn').get_parameter_value().double_value
        self.accel = self.get_parameter('max_accel').get_parameter_value().double_value
        self.deccel = self.get_parameter('max_deccel').get_parameter_value().double_value
        self.speed = 0.0
        self.turn = 0.0
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.pub_twist = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
        self.pub_key = self.create_publisher(String, 'key_pressed', 10)

        self.get_logger().info('Started teleop')
        self.save_terminal_settings()

        self.move_bindings = {
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

        self.halt_bindings = {
            'k': (0, 0, 0, 0),
            'K': (0, 0, 0, 0),
            '': (0, 0, 0, 0),
        }

        self.emergency_halt_bindings = {
            ' ': (0, 0, 0, 0),  # space bar
        }

        self.speed_bindings = {
            'q': (1.1, 1.1),
            'z': (.9, .9),
            'w': (1.1, 1),
            'x': (.9, 1),
            'e': (1, 1.1),
            'c': (1, .9),
        }

        self.print_help_message()

        self.last_key = ''
        self.read_key_timeout = 0.1

        self.key_lock = threading.Condition()
        self.shutdown = threading.Event()
        self.emergency_stop = threading.Event()
        self.stopped_accel_runtime = threading.Event()
        self.stopped_accel_runtime.set()

        self.eval_key_thread = threading.Thread(target=self.evaluate_key, name='self.eval_key_thread')
        self.read_key_thread = threading.Thread(target=self.read_key, name='getKeyThread')        
        self.eval_key_thread.start()
        self.read_key_thread.start()
        rclpy.spin(self)
        

    def parameters_callback(self, params):
        for param in params:
            if param.name == "speed":
                self.target_speed = param.value
            if param.name == "turn":
                self.target_turn = param.value
            if param.name == "max_accel":
                self.accel = param.value
        return SetParametersResult(successful=True)


    ''' Updates the ROS-parameters after the local speed and turn are changed
    '''
    def update_ros_params(self):
        param_speed = Parameter('target_speed', Parameter.Type.DOUBLE, self.target_speed)
        param_turn = Parameter('target_turn', Parameter.Type.DOUBLE, self.target_turn)
        self.set_parameters([param_speed, param_turn])
    

    ''' Deccelerate to velocity=zero
    '''
    def deccelerate_to_halt(self, key_copy):
        if self.last_cmd_halt:
            return
        
        stop_time = time.time()
        v0 = self.speed
        t0 = self.turn
        self.stopped_accel_runtime.clear()
        while abs(self.speed) > 0.0:
            twist = self.get_accelerated_twist(stop_time, key_copy, 0.0, 0.0, self.deccel, v0, t0)
            self.pub_twist.publish(twist)

            if self.emergency_stop.is_set():
                self.stopped_accel_runtime.set()
                return
        
        self.print_clean('stop')
        self.stopped_accel_runtime.set()
        self.last_cmd_halt = True
    

    def send_emergency_halt(self):
        self.emergency_stop.set()
        self.stopped_accel_runtime.wait()
                
        self.speed = 0.0
        self.turn = 0.0
        
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_twist.publish(twist)
        self.print_help_message()
        self.print_clean('\nemergency stop')
        self.last_cmd_halt = True

        self.emergency_stop.clear()
    

    def print_clean(self, msg):
        # change terminal settings temporarily for standard printing
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        print(msg)
        # default state: setup terminal for read_key
        tty.setraw(sys.stdin.fileno())
    

    def print_target_speeds(self):
        p = (self.target_speed, self.target_turn, self.accel, self.deccel)
        msg = 'speeds:\tlinear %.2f   turn %.2f    max_accel: %.2f   max_deccel: %.2f' % p
        self.print_clean(msg)
    
    
    def print_help_message(self):
        help_message = """
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

        CTRL+C to quit, Space for EMERGENCY HALT
        """
        self.print_clean(help_message)
        self.print_target_speeds()
    

    def save_terminal_settings(self):
        if sys.platform == 'win32':
            return None
        self.terminal_settings = termios.tcgetattr(sys.stdin)


    def restore_terminal_settings(self):
        if sys.platform == 'win32':
            return
        old_settings = self.terminal_settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


    def get_accelerated_twist(self, start_move_time, key_copy, target_speed, target_turn, accel, v0, t0):
        min_speed = 0.005
        timeSinceHalt = time.time() - start_move_time
        self.speed = timeSinceHalt * accel + v0
        self.turn = timeSinceHalt * accel + t0

        if self.speed >= target_speed and accel>0:
            self.speed = target_speed
        if self.turn >= target_turn and accel>0:
            self.turn = target_turn
        
        if self.speed < min_speed:
            self.speed = 0.0
        if self.turn < min_speed:
            self.turn = 0.0
        
        x = self.move_bindings[key_copy][0]
        y = self.move_bindings[key_copy][1]
        z = self.move_bindings[key_copy][2]
        th = self.move_bindings[key_copy][3]
        
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = x * self.speed
        twist.linear.y = y * self.speed
        twist.linear.z = z * self.speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th * self.turn        
        return twist

    def read_key(self):
        key_changed = False
        key = '' # local read variable for key

        while True:  
            if sys.platform == 'win32':
                # getwch() returns a string on Windows    
                key = msvcrt.getwch()
            else:
                tty.setraw(sys.stdin.fileno())            
                key = sys.stdin.read(1) # this is blocking
                with self.key_lock:
                    self.last_key = key
                    self.key_lock.notify()
                # msg = String()
                # msg.data = 'last key pressed:' + self.last_key
                # self.pub_key.publish(msg)

            if key in self.emergency_halt_bindings.keys(): # space-bar
                self.send_emergency_halt()
            
            if key == "\x03": # ctrl+c
                self.send_emergency_halt()
                self.shutdown.set()
                break
        
        self.eval_key_thread.join()
        self.shut_down_node()
    

    def evaluate_key(self):
        x = 0.0
        y = 0.0
        z = 0.0
        th = 0.0
        help_msg_status = 0.0
        print_msg_every = 6
        self.last_cmd_halt = False
        start_move_time = time.time()
        last_move_binding_key = 'i'
        
        
        while True:
            with self.key_lock:
                self.key_lock.wait(timeout=self.read_key_timeout)
                key_copy = self.last_key
                self.last_key = ''
            
            if key_copy in self.move_bindings.keys():
                if self.last_cmd_halt:
                    self.print_help_message()
                    self.print_clean('\nmoving...')
                    start_move_time = time.time()
                self.last_cmd_halt = False

                v0 = self.speed
                t0 = self.turn
                twist = self.get_accelerated_twist(
                    start_move_time, key_copy, 
                    self.target_speed, self.target_turn, 
                    self.accel, v0, t0
                )
                last_move_binding_key = key_copy
                self.pub_twist.publish(twist)
            
            elif key_copy in self.speed_bindings.keys():
                self.target_speed = self.target_speed * self.speed_bindings[key_copy][0]
                self.target_turn = self.target_turn * self.speed_bindings[key_copy][1]
                self.update_ros_params()

                self.print_help_message()
                self.print_clean('\nstop')
                self.deccelerate_to_halt(last_move_binding_key)
            
            elif key_copy in self.halt_bindings.keys():
                self.deccelerate_to_halt(last_move_binding_key)

            if (key_copy == 'r'):
                self.target_speed = 0.25
                self.target_turn = 0.25
                self.print_help_message()
                self.print_clean('\nreset target speeds')
                self.update_ros_params()
            
            if self.shutdown.is_set():
                break
    

    def shut_down_node(self):
        self.send_emergency_halt()
        self.restore_terminal_settings()
        self.get_logger().info('Sent STOP twist message, shutting down node')
        self.destroy_node()
        rclpy.shutdown()



def main():
    rclpy.init()
    node = TeleopKeyboard()
    


if __name__ == '__main__':
    main()
