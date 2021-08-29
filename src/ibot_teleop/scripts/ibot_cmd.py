#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control mbot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""
moveBindings = {
        'w':(1,0),
        'e':(1,-1),
        'a':(0,1),
        'd':(0,-1),
        'q':(1,1),
        's':(-1,0),
        'z':(-1,1),
        'c':(-1,-1),
        'r':(1,0), #逆时针旋转
        'f':(-1,0), #顺时针旋转
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

def vels(speed_x,speed_y,turn):
    return "currently:\tspeed_x %s,speed_y %s\tturn %s " % (speed_x,speed_y,turn)

def limit(num,max_num,min_num):
    if num > max_num:
        num=max_num
    elif num < min_num:
        num=min_num
    return num

speed_x = 0.5
speed_y = 0.5
speed_addOnce = 2
turn = 0.2
turn_addOnce = 1

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('ibot_cmd')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    speed_dir_x=0
    speed_dir_y=0
    turn_dir=0
    control_speed_x=0.0
    control_speed_y=0.0
    control_turn=0
    run=0
    count = 0
    try:
        print msg
        while(1):
            key   = getKey()
            twist = Twist()
            if key in moveBindings.keys():
                run=1
                count=0
                if key == 'r' or key == 'f': 
                    speed_dir_x=0
                    speed_dir_y=0
                    turn_dir = moveBindings[key][0]
                else:
                    turn_dir=0
                    speed_dir_x = moveBindings[key][0]
                    speed_dir_y = moveBindings[key][1]
            elif key == ' ':
                speed_dir_x=0
                speed_dir_y=0
                turn_dir=0
                control_speed_x=0
                control_speed_y=0
                control_turn=0
                run=0
            elif key == 'u':
                speed_x = speed_x+speed_addOnce
                speed_y = speed_y+speed_addOnce
                run=0
            elif key == 'i':
                speed_x = speed_x-speed_addOnce
                speed_y = speed_y-speed_addOnce
                run=0     
            elif key == 'j':
                turn = turn+turn_addOnce
                run=0
            elif key == 'k':
                turn = turn-turn_addOnce     
                run=0
            else:
                count = count + 1
                if count > 5:         #没有count会存在一定的空隙
                    run = 0
                if (key == '\x03'): #for ctrl + c exit
                    break

            if run == 1:
                control_speed_x=speed_x*speed_dir_x
                control_speed_y=speed_y*speed_dir_y
                control_turn=turn*turn_dir
            else:
                control_speed_x=0
                control_speed_y=0
                control_turn=0 

            control_speed_x=limit(control_speed_x,30,-30)
            control_speed_y=limit(control_speed_y,30,-30)
            control_turn=limit(control_turn,10,-10)    

            twist.linear.x = control_speed_x; twist.linear.y = control_speed_y; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)
            print vels(control_speed_x,control_speed_y,control_turn)
    except Exception as e:
        print e
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)