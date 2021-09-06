# teleop_twist_keyboard
A fork of the [Generic Keyboard Teleoperation for ROS](https://github.com/ros2/teleop_twist_keyboard). This teleop-keyboard features a ROS parameter interface as well as intuitive keyboard operation through press-and-hold.

## Launch

To launch in seperate terminal window: 
```
ros2 launch teleop_twist_keyboard teleop_twist_keyboard_launch.py
```
The two parameters ``speed`` and ``turn`` can be interfaced through ROS parameters. Initial values are set to 0.25 m/s². For custom initial values, change launch file or run with  
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=3.141 -p turn:=3.141
```

## Usage

If no key is pressed or the most recent key has been released, the stop command is sent.

```
This node takes keypresses from the keyboard and publishes them as Twist
messages. It works best with a US keyboard layout.
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
```
