#! /usr/bin/env python
"""
import rospy
from math import atan2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion

x = 1.0062517078
y = 3.5000025917
t = 0.0	#Theta
i = 0

rospy.init_node("Odom_and_Obstacle")

r = rospy.Rate(4)

speed = Twist()
goal = Point()

goalsx = [ 1.0, 1.0, 1.5, 1.5]
goalsy = [ 3.5, 4.0, 4.0, 4.5]

def coords(msg):
    global x
    global y
    global t
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (_,_,t) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    

def obstacle(msg):
	print msg.ranges[20]
	#if msg.ranges[360] < 1:
	#	speed.linear.x = 0.0
	#	speed.angular.z = 0.0
	#	print("Obstacle")

#sub1 = rospy.Subscriber("/tb3_hsc/odom", Odometry, coords)
sub1 = rospy.Subscriber("/odom", Odometry, coords)
#sub2 = rospy.Subscriber("/tb3_hsc/scan", LaserScan, obstacle)
#pub = rospy.Publisher("/tb3_hsc/cmd_vel", Twist, queue_size = 1)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

def setgoal(npoint):
	print(npoint)
	global i
	global goal
	global goalsx
	if(npoint >= len(goalsx)):
		i = 0
	goal.x = goalsx[i]
	goal.y = goalsy[i]
	#print("X: " + str(goal.x))
	#print("Y: " + str(goal.y))

while not rospy.is_shutdown():
	setgoal(i)
	#Difference between goal and actual position
	inc_x = goal.x - x	
	inc_y = goal.y - y
	
	angle_to_goal = atan2(inc_y, inc_x)
    
    
	if abs(angle_to_goal - t) > 0.1:
		speed.linear.x = 0.0
		speed.angular.z = 0.2
		if(abs(inc_x) <= 0.05 and abs(inc_y) <= 0.05):
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			print("Reached point xyz")
			global i
			i = i + 1
			setgoal(i)
			
	else:
		speed.linear.x = 0.2
		speed.angular.z = 0.0
		if(abs(inc_x) <= 0.05 and abs(inc_y) <= 0.05):
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			print("Reached point ang")
			global i
			i = i + 1
			setgoal(i)

    	
	pub.publish(speed)
	r.sleep()
 """

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

group_variable_values = group.get_current_joint_values()

if group.has_end_effector_link() == True :
	eef_link = group.get_end_effector_link()
	print("============= End effetor link: %s" % eef_link)
else :
	print("============= Not end effector link detected")

#Plan home
group_variable_values[0] = 0
group_variable_values[1] = 0
group_variable_values[2] = 0
group_variable_values[3] = 0
group.set_joint_value_target(group_variable_values)
plan_home = group.plan()
group.go(wait=True)
rospy.sleep(5)
print("Plan home finished")


#Plan pick paso 1
group_variable_values[0] = 0 	#Brazo Izq +0, Der -0
#group_variable_values[1] = 0.5	#Brazo adelante 0 pos inicial hasta 0.7
#group_variable_values[2] = 1	#L arriba 0, 1 abajo
group_variable_values[3] = 0.3
group.set_joint_value_target(group_variable_values)

group.set_end_effector_link(eef_link)

plan_pick1 = group.plan()
group.go(wait=True)
rospy.sleep(5)

print("Plan pick 1 finished")

#Plan pick paso 2
group_variable_values[0] = 0 	#Brazo Izq +0, Der -0
#group_variable_values[1] = 0.5	#Brazo adelante 0 pos inicial hasta 0.7
group_variable_values[2] = 0.5	#L arriba 0, 1 abajo
#group_variable_values[3] = 0.3
group.set_joint_value_target(group_variable_values)

plan_pick2 = group.plan()
group.go(wait=True)
rospy.sleep(5)

print("Plan pick 2 finished")

#Plan pick paso 3
group_variable_values[0] = 0 	#Brazo Izq +0, Der -0
#group_variable_values[1] = 0.5	#Brazo adelante 0 pos inicial hasta 0.7
#group_variable_values[2] = 0.5	#L arriba 0, 1 abajo
group_variable_values[3] = 0.3
group.set_joint_value_target(group_variable_values)

group.set_end_effector_link(eef_link)

plan_pick2 = group.plan()
group.go(wait=True)
rospy.sleep(5)

print("Plan pick 3 finished")
moveit_commander.roscpp_shutdown()

