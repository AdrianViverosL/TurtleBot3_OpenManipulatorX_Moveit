#! /usr/bin/env python


import sys
import copy
import rospy
from math import atan2
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion

x = 1.0001030
y = 2.5001069
t = 0.0		#Theta
i = 0		#Coords number
p = 0		#Plan number

f_pick = False	#Flag location of pickup reached
f_goal = False	#Flag location of non-pickup reached 

rospy.init_node("Odom_and_Obstacle", anonymous=True)

r = rospy.Rate(4)

speed = Twist()
goal = Point()

goalsx = [ 1.0, 1.0, 1.0]
goalsy = [ 3.0, 3.5, 4.5]

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

group_variable_values = group.get_current_joint_values()

def coords(msg):
    global x
    global y
    global t
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (_,_,t) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    
sub1 = rospy.Subscriber("/odom", Odometry, coords)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

def setPlan(nPlan):
	#Link 0 Brazo Izq +0, Der -0
	#Link 1 Brazo adelante 0 pos inicial hasta 0.7
	#Link 2 L arriba 0, 1 abajo
	#Link 3 Grip arriba abajo
	rospy.sleep(1)
	if(nPlan == 0):
		print("Plan home enter")
		#Plan home		
		group_variable_values[0] = 0
		group_variable_values[1] = 0.1
		group_variable_values[2] = 0
		group_variable_values[3] = 0.1
		group.set_joint_value_target(group_variable_values)
		plan_home = group.plan()
		#group.go(wait=True)
		group.execute(plan_home,wait=True)
		rospy.sleep(5)
		#moveit_commander.roscpp_shutdown()
		print("Plan home finished")
	
	elif(nPlan == 1):
		print("Plan pick enter")
		#Plan pick		
		group_variable_values[0] = 0
		group_variable_values[1] = 0.5
		group_variable_values[2] = 0
		group_variable_values[3] = 0.5
		group.set_joint_value_target(group_variable_values)
		plan_pick1 = group.plan()
		#group.go(wait=True)
		group.execute(plan_pick1,wait=True)
		rospy.sleep(5)
		#moveit_commander.roscpp_shutdown()
		print("Plan pick finished")
		
	
def setgoal(npoint):
	global i
	global goal
	global goalsx

	if(npoint >= len(goalsx)):
		i = 0

	goal.x = goalsx[i]
	goal.y = goalsy[i]


while not rospy.is_shutdown():
	setgoal(i)	
	#Difference between goal and actual position
	inc_x = goal.x - x	
	inc_y = goal.y - y
	
	angle_to_goal = atan2(inc_y, inc_x)

	if abs(angle_to_goal - t) > 0.2:
		speed.linear.x = 0.0
		speed.angular.z = 0.5
		if(abs(inc_x) <= 0.05 and abs(inc_y) <= 0.05):
			#speed.linear.x = 0.0
			#speed.angular.z = 0.0
			#pub.publish(speed)
			#r.sleep()
			print("Reached point xyz: " + str(i))
			if(i == 1): #or i == otro or ....
				f_pick = True
			i = i + 1
			setgoal(i)
			f_goal = True
			
			
	else:
		speed.linear.x = 0.2
		speed.angular.z = 0.0
		if(abs(inc_x) <= 0.05 and abs(inc_y) <= 0.05):
			#speed.linear.x = 0.0
			#speed.angular.z = 0.0
			#pub.publish(speed)
			#r.sleep()
			print("Reached point ang: " + str(i))
			if(i == 1): #or i == otro or ....
				f_pick = True
			i = i + 1
			setgoal(i)
			f_goal = True
			

	#No ha llegado a coord punto
	if(f_goal != True):
		pub.publish(speed)
		r.sleep()
	#Llego a coord con obj a recoger
	#Alto, pick y return home
	elif(f_goal == True and f_pick == True):
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		pub.publish(speed)
		setPlan(1)
		setPlan(0)
		f_goal = False
		f_pick = False
	#Llego a un punto sin objeto a recoger
	#Alto y sigo a otro punto
	elif(f_goal == True and f_pick == False):
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		pub.publish(speed)
		rospy.sleep(1)
		f_goal = False


