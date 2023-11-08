#!/usr/bin/env python
# coding:utf-8
import rospy

from geometry_msgs.msg import TwistStamped

import sys
import select
import termios
import tty

msg = """
Control Your CDPR!(simple version)
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

q/e : turn anti-clockwise/clockwise
w/s : move forward/retreat
a/d : move left/right
u/j : increase/decrease speed of moving forward and retreat by 10%
i/k : increase/decrease speed of moving left and right by 10%
o/l : increase/decrease angular speed by 10%
 c  : force stop

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, -1, 0),
    'd': (0, 1, 0),
    'e': (0, 0, -1),
    'q': (0, 0, 1),
}

speedBindings = {
    'u': (1.1, 1, 1),
    'j': (.9, 1, 1),
    'i': (1, 1.1, 1),
    'k': (1, .9, 1),
    'o': (1, 1, 1.1),
    'l': (1, 1, .9),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(x_vel, y_vel, turn):
    return "currently:  speed in x axis %.3f, in y axis %.3f, turn speed %.3f " % (x_vel, y_vel, turn)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('cdpr_keyboard_teleop')
    pub = rospy.Publisher('/cmd_chassis', TwistStamped, queue_size=5)

    x_vel = rospy.get_param("~x_vel", 0.5)
    y_vel = rospy.get_param("~y_vel", 0.5)
    turn = rospy.get_param("~turn", 0.2)
    max_x_vel = rospy.get_param("~max_x_vel", 2)
    max_y_vel = rospy.get_param("~max_y_vel", 2)
    max_turn = rospy.get_param("~max_turn", 1)
    x_acc = rospy.get_param("~x_acc", 0.1)
    y_acc = rospy.get_param("~y_acc", 0.1)
    turn_acc = rospy.get_param("~turn_acc", 0.02)

    x = 0
    y = 0
    t = 0
    status = 0
    count = 0

    target_x_vel = 0
    target_y_vel = 0
    target_turn = 0
    control_x_vel = 0
    control_y_vel = 0
    control_turn = 0

    try:
        print(msg)
        print(vels(x_vel, y_vel, turn))
        print("max:  x_vel %s   y_vel %s   turn %s " %
              (max_x_vel, max_y_vel, max_turn))
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                t = moveBindings[key][2]
                count = 0
            elif key in speedBindings.keys():
                x_vel = x_vel * speedBindings[key][0]
                y_vel = y_vel * speedBindings[key][1]
                turn = turn * speedBindings[key][2]
                count = 0
                if x_vel > max_x_vel:
                    print(
                        "x velocity has increased to its maximum, stop increasing!")
                    x_vel = max_x_vel
                if y_vel > max_y_vel:
                    print(
                        "y velocity has increased to its maximum, stop increasing!")
                    y_vel = max_y_vel
                if turn > max_turn:
                    print(
                        "turn velocity has increased to its maximum, stop increasing!")
                    turn = max_turn
                print(vels(x_vel, y_vel, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == 'c':
                x = 0
                y = 0
                t = 0
                control_x_vel = 0
                control_y_vel = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    y = 0
                    t = 0
                    if (key == '\x03'):
                        break

            # 限速
            target_x_vel = constrain(x_vel * x, -max_x_vel, max_x_vel)
            target_y_vel = constrain(y_vel * y, -max_y_vel, max_y_vel)
            target_turn = constrain(turn * t, -max_turn, max_turn)

            # 速度平滑：当前速度小于目标速度时，当前速度递增，直至相等；当前速度大于目标速度时，当前速度递减，直至相等
            # control_x_vel = makeSimpleProfile(
            #     control_x_vel, target_x_vel, x_acc)
            #  = makeSimpleProfile(
            #     control_y_vel, target_y_vel, y_acc)
            # control_turn = makeSimpleProfile(
            #         control_turn, target_turn, turn_acc)
            control_x_vel = target_x_vel
            control_y_vel = target_y_vel
            control_turn = target_turn

            twist_msg = TwistStamped()
            twist_msg.twist.linear.x = control_x_vel
            twist_msg.twist.linear.y = control_y_vel
            twist_msg.twist.linear.z = 0
            twist_msg.twist.angular.x = 0
            twist_msg.twist.angular.y = 0
            twist_msg.twist.angular.z = control_turn

            twist_msg.header.stamp = rospy.Time.now()

            pub.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = 0
        twist_msg.twist.linear.y = 0
        twist_msg.twist.linear.z = 0
        twist_msg.twist.angular.x = 0
        twist_msg.twist.angular.y = 0
        twist_msg.twist.angular.z = 0

        twist_msg.header.stamp = rospy.Time.now()

        pub.publish(twist_msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
