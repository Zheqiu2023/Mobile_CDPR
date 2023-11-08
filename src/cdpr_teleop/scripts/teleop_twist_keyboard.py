#!/usr/bin/env python
# coding:utf-8

from __future__ import print_function

import threading

import rospy

from geometry_msgs.msg import TwistStamped

import sys
import termios
import tty
from select import select


TwistMsg = TwistStamped

msg = """
Control Your CDPR!
Reading from the keyboard and Publishing to Twist!
---------------------------
Holonomic mode (strafing):
   q    w    e
   a    s    d
   z    x    c

For Steering mode, hold down the shift key:
---------------------------
   Q    E
   A    D
   Z    C

anything else : stop

u/j : increase/decrease only linear speed in x axis by 10%
i/k : increase/decrease only linear speed in y axis by 10%
o/l : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0),
    'a': (0, 1, 0),
    's': (0, 0, 0),
    'd': (0, -1, 0),
    'x': (-1, 0, 0),
    'q': (1, 1, 0),
    'e': (1, -1, 0),
    'z': (-1, 1, 0),
    'c': (-1, -1, 0),

    'A': (0, 0, 1),
    'D': (0, 0, -1),
    'Q': (1, 0, 1),
    'E': (1, 0, -1),
    'Z': (-1, 0, 1),
    'C': (-1, 0, -1),
}

speedBindings = {
    'u': (1.1, 1, 1),
    'j': (.9, 1, 1),
    'i': (1, 1.1, 1),
    'k': (1, .9, 1),
    'o': (1, 1, 1.1),
    'l': (1, 1, .9),
}


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher(
            '/cmd_chassis', TwistMsg, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(
                    self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception(
                "Got shutdown request before subscribers connected")

    def update(self, x, y, th, x_vel, y_vel, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.th = th
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()
        twist_msg.header.stamp = rospy.Time.now()

        while not self.done:
            twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist_msg.twist.linear.x = self.x * self.x_vel
            twist_msg.twist.linear.y = self.y * self.y_vel
            twist_msg.twist.linear.z = 0
            twist_msg.twist.angular.x = 0
            twist_msg.twist.angular.y = 0
            twist_msg.twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist_msg.twist.linear.x = 0
        twist_msg.twist.linear.y = 0
        twist_msg.twist.linear.z = 0
        twist_msg.twist.angular.x = 0
        twist_msg.twist.angular.y = 0
        twist_msg.twist.angular.z = 0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(x_vel, y_vel, turn):
    return "currently linear speed: x %.3f  y %.3f, angular speed: %.3f" % (x_vel, y_vel, turn)


if __name__ == "__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    x_vel = rospy.get_param("~x_vel", 0.5)
    y_vel = rospy.get_param("~y_vel", 0.5)
    turn = rospy.get_param("~turn", 0.2)
    x_vel_limit = rospy.get_param("~x_vel_limit", 2)
    y_vel_limit = rospy.get_param("~y_vel_limit", 2)
    turn_limit = rospy.get_param("~turn_limit", 1)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, th, x_vel, y_vel, turn)

        print(msg)
        print(vels(x_vel, y_vel, turn))
        while (1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
            elif key in speedBindings.keys():
                x_vel = min(x_vel_limit, x_vel * speedBindings[key][0])
                y_vel = min(y_vel_limit, y_vel * speedBindings[key][1])
                turn = min(turn_limit, turn * speedBindings[key][2])
                if x_vel == x_vel_limit:
                    print("Linear speed limit in x axis reached!")
                if x_vel == x_vel_limit:
                    print("Linear speed limit in y axis reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(x_vel, y_vel, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already stopped.
                if key == '' and x == 0 and y == 0 and th == 0:
                    continue
                x = 0
                y = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, th, x_vel, y_vel, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
