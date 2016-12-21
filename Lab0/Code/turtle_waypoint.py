#!/usr/bin/env python
#import roslib; roslib.load_manifest('lab0_ros')
import rospy

#For command line arguments
import sys
#For atan2
import math

#TODO: Import the messages we need
import turtlesim.msg
from geometry_msgs.msg import Twist

#Initialization of turtle position
x=None
y=None
theta=None

#Position tolerance for both x and y
tolerance=0.1
#Have we received any pose msg yet?
gotPosition=False

def callback(pose_msg):
    global x,y,theta,gotPosition
    #TODO:Store the position in x,y and theta variables.
    #rospy.init_node('callback',anonymous=True)    
    #rospy.Subscriber("turtle1/pose", turtlesim.msg.Pose,callback)
    x=pose_msg.x
    y=pose_msg.y
    theta=pose_msg.theta
    gotPosition=True

def waypoint():
    global gotPosition
    #TODO: Define the pulisher: Name of the topic. Type of message
    pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=15)
    

    #Name of the node
    rospy.init_node('turtle_waypoint',anonymous=True)
    #TODO: Define the subscriber: Name of the topic. Type of message. Callback function
    rospy.Subscriber("/turtle1/pose", turtlesim.msg.Pose, callback)

    #Has the turtle reach position?
    finished=False
    #If the point hasn't been specified in a command line:
    if(len(sys.argv)!=3):
        print('X and Y values not set or not passed correctly. Looking for default parameters.')
        #TODO: If ROS parameters default_x and default_y exist:
        if (rospy.has_param('default_x') and rospy.has_param('default_y') ): #Change this for the correct expression
            #TODO: Save them into this variables
            x_desired=rospy.get_param('default_x') #Change this for the correct expression
            y_desired=rospy.get_param('default_y')#Change this for the correct expression
            print('Heading to: %f,%f' %(x_desired, y_desired))
        else:
            print('Default values parameters not set!. Not moving at all')
            finished=True
    else:
        #Save the command line arguments.
        x_desired=float(sys.argv[1])
        y_desired=float(sys.argv[2])
        print('Heading to: %f,%f' %(x_desired, y_desired))

    while not rospy.is_shutdown() and not finished:
        if(gotPosition):
            #TODO: Send a velocity command for every loop until the position is reached within the tolerance.
            twist = Twist() 
            twist.linear.x=2*math.sqrt(math.pow((x-x_desired),2)+math.pow((y-y_desired),2) )            
            twist.linear.y=0 
            twist.linear.z=0            
            twist.angular.x=0 
            twist.angular.y=0
            twist.angular.z=math.atan2(y_desired-y,x_desired-x)-theta
            if(twist.angular.z<-1*math.pi):
                twist.angular.z=twist.angular.z+2*math.pi
            elif(twist.angular.z>math.pi):
                twist.angular.z=twist.angular.z-2*math.pi
                
            twist.angular.z=4*twist.angular.z    
            pub.publish(twist)
            if(math.sqrt(math.pow((x-x_desired),2)+math.pow((y-y_desired),2))<tolerance): #Change for the correct expression
                finished=True        

        #Publish velocity commands every 0.3 sec.
        rospy.sleep(0.3)

if __name__ == '__main__':
    try:
        waypoint()
    except rospy.ROSInterruptException:
        pass
    
    
    
    
