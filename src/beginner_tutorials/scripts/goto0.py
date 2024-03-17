#!/usr/bin/env python3

import signal
import math
import rospy
import sys
import os
import time

# TODO make sure the rot. direction is correct in the real world.

from nav_msgs.msg import Odometry
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import Point, Vector3, Twist

# constants
POS_THRESH = 0.1
THETA_THRESH = 0.1

MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 0.5

# odometry globals
# robot's x position
posx = 0.0

# robot's y position
posy = 0.0

# robot's angle in radians from the horizontal
theta = 0.0

def read_locs(filename):
    """ Reads in a list of 2d locations for the robot to travel to in order """
    with open(filename) as f:
        lines = f.readlines()
        locs = [ x.split() for x in lines ]
        locs = [ Point(float(x), float(y), 0.0) for [x, y] in locs ]
        return locs

def quat2zrot(q):
    """ Parses Z rotation from a quaternion """
    return 2 * math.atan2(q.z, q.w)


def upd_odom(data):
    """ Updates node-global odometry data """
    global posx
    global posy
    global theta

    posx = data.pose.pose.position.x
    posy = data.pose.pose.position.y
    theta = quat2zrot(data.pose.pose.orientation)


def main():
    rospy.init_node('goto', anonymous=True)

    # read in the locations
    locs = read_locs(sys.argv[1])

    rospy.Subscriber('/pose', Odometry, upd_odom)

    # enables the motors
    motor_state_pub = rospy.Publisher('/cmd_motor_state', MotorState, 
                                      queue_size=30, latch=True)
    motor_state_pub.publish(MotorState(1))

    twist_pub = rospy.Publisher('/cmd_vel', Twist,
                                queue_size=30, latch=True)

    # message to send to stop the robot
    stop_twist = Twist(Vector3(0,0,0), Vector3(0,0,0))

    #cmd = Twist()
    #cmd.angular.z = 0.1
    #twist_pub.publish(cmd)

    #time.sleep(3)

    def handlec(sig, frame):
        sys.exit(0)

    signal.signal(signal.SIGINT, handlec)


    for goal in (locs * 2):

        print("navigating towards", goal)

        dx = goal.x - posx
        dy = goal.y - posy

        # how far from the goal we are
        dpos = math.sqrt(dx ** 2 + dy ** 2)

        # continue until we are "near" the goal
        while dpos > POS_THRESH: 

            dx = goal.x - posx
            dy = goal.y - posy

            # how far from the goal we are
            dpos = math.sqrt(dx ** 2 + dy ** 2)

            cmd = Twist()

            # difference in angle from the x-axis
            dtheta = math.atan2(dy, dx) - theta

            if dtheta > math.pi:
                dtheta = -(2*math.pi - dtheta)
            elif dtheta < -math.pi:
                dtheta = 2*math.pi + dtheta 

            #print("curtheta", theta)
            #print("dtheta:", dtheta)
            # if we are pointing the way wrong way, adjust
            if abs(dtheta) > THETA_THRESH:
                if dtheta < 0:
                    cmd.angular.z = max(-MAX_ANG_VEL, dtheta)
                else:
                    cmd.angular.z = min(MAX_ANG_VEL, dtheta)
                #print("turning", cmd.angular.z)
            # otherwise full steam ahead
            else:
                cmd.linear.x = min(MAX_LIN_VEL, dpos)

            # send out the command
            twist_pub.publish(cmd)

            # give ros some breathing room
            rospy.Rate(10).sleep()

        twist_pub.publish(stop_twist)
        
    print("Traversed each point specified in file:", sys.argv[1])

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
