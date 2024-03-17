#!/usr/bin/env python
"""
file: goto.py
description:    Robot goes to specified points and prints a statement when close
language: python3
authors:    Sam Winebrake   sgw1122@rit.edu
"""

import sys
import rospy
import math
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
velocity_publisher = None
motor_publisher = None
odometry_subscriber = None
rate = None

x = None
y = None
q = None
theta = None
locations = []
cur_location = None
DISTANCE_ERROR = .01
list_location = 0


def processFile():
    global locations
    
    # read in file and make list of tuples
    filename = sys.argv[1]
    
    # filename will be text with x,y pairs on new lines, space separating
    with open(filename) as file:
        for line in file:
            # take in line of file and split by space
            coords = line.split()

            # turn into x,y floats
            xy_pair = (float(coords[0]), float(coords[1]))

            # add to locations that are returned
            locations.append(xy_pair)
    print("Locations to go to: ")
    print(locations)


def distance(goal_x, goal_y, robot_x, robot_y):
    # euclidean formula
    return math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)


def cur_pos(data):
    # callback function
    global x, y, q, theta, list_location, locations, odometry_subscriber

    # get current goal location
    cur_goal = locations[list_location]
    goal_x = cur_goal[0]
    goal_y = cur_goal[1]

    # get global cur position of robot
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q = data.pose.pose.orientation
    
    

    vel_msg = Twist()

    theta = 2*math.atan2(q.z, q.w)
    gamma = math.atan2(goal_y - y, goal_x - x)

    # trying to minimize theta2 - theta... this will be angle between robot and goal point. get within .1
    difference_angle = (gamma - theta) % math.tau
    print("da: " + str(difference_angle))
    print("distance: " + str(distance(goal_x, goal_y, x, y)))

    if distance(goal_x, goal_y, x, y) >= DISTANCE_ERROR:
        if difference_angle > .1 or difference_angle < -.1:
            # can fix later to be more efficient, give angular velocity and publish
            vel_msg.linear.x =0
            # if 2pi - dif angle is smaller than dif angle, then turn negative way
            #if math.tau - difference
            if 0 <= difference_angle <= math.pi or -(math.pi) >= difference_angle >= (-2*math.pi):
            	vel_msg.angular.z = .1
            else:
            	vel_msg.angular.z = -.1
            velocity_publisher.publish(vel_msg)
            rate.sleep()
        else:
            # if facing right direction, give linear velocity and publish
            vel_msg.angular.z = 0
            vel_msg.linear.x = .1
            velocity_publisher.publish(vel_msg)
            rate.sleep()
    else:
        # stop the robot
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        print("Robot has reached location x: " + str(goal_x) + ", y: " + str(goal_y))

        # change index of list
        if list_location < (len(locations)-1):
            list_location += 1
        else:
            print("Complete")
            odometry_subscriber.unregister()
            exit()


def init_node():
    global velocity_publisher, odometry_subscriber, rate
    # global motor_publisher
    rospy.init_node('goto', anonymous=True)

    # publishers
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    motor_publisher = rospy.Publisher('/cmd_motor_state', MotorState, latch=True, queue_size=10)

    # # set motor state
    motor = MotorState()
    # # set the motor state
    motor.state = 1
    motor_publisher.publish(motor)

    odometry_subscriber = rospy.Subscriber('/pose', Odometry, cur_pos)
    rate = rospy.Rate(10)


if __name__ == '__main__':
    try:
        processFile()
        init_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


