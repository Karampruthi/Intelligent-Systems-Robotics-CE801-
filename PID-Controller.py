#!/usr/bin/env python

# PID 


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

call = 0 
e_prev = 0.0  
e_i = 0.0
last_right_sensor_reading = 0.8
speed_max = 0.5 
setpoint = 0.6
Kp = 1.0
Kd = 20  #  To stabalize wave moment of rosbot kd is higher
Ki = 0.1
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # creates publisher class


# function to control velocity of robot
def forwards(speed, turn):
	global pub
	vel_x = Twist() # initiates twist message
	vel_x.linear.x = speed  # sets linear speed of robot
	vel_x.angular.z = turn  # sets angle to turn to pid function output (turns left?)
	pub.publish(vel_x) #publishes velocity

def pid(sensor_value):    # to get the value of angular velocity from PID equation
	global e_prev , e_i
	global speed_max
	global setpoint
	global Kp , Kd , Ki

	
	e_p = setpoint - sensor_value  
	e_i = []   
   

	if len(e_i) > 9:
		e_i.pop(0)
	e_i.append(e_p)
	e_I = sum(e_i)   # e_i = sum of last 9 values of ep 

	rospy.loginfo(" error %f",e_p)
	e_d = e_p - e_prev
	e_prev = e_p
	control_val = Kp * e_p + Kd * e_d + Ki * e_I 
	
	rospy.loginfo("control value %f", control_val)
	return control_val    

    

def callback(msg):
	global last_right_sensor_reading
	global call
	call = 1
	right_values = []
	for i in range(-1,2):
		dist = msg.ranges[540 + 60 *i]
		rospy.loginfo("distance %f",dist)
		right_values.append(dist)
	min_dist = min(right_values)    # get minimum value  
	if min_dist > 1.0:              # to solve the problem of infinity in the equation taking maximum sensor value = 1
		min_dist = 1.0
	

	last_right_sensor_reading = min_dist 
	rospy.loginfo("sensor reading %f", last_right_sensor_reading)
	if last_right_sensor_reading > 2:
		stop()
	
def controller():   # sending linear and angular velocity after processing PID equation  
	global last_right_sensor_reading
	rate = rospy.Rate(10)  # sleep in loop at rate 25hz
	base_speed = 0.4 
	while not rospy.is_shutdown():
		
		pid_value = pid(last_right_sensor_reading)
		forwards(base_speed, pid_value)
		rate.sleep()  # pauses rate

def stop() :    # adding a function to stop rosbot after pressing (cntr+C)
    vel = Twist()
    vel.linear.x = 0.0 
    vel.angular.z = 0.0 
    pub.publish(vel)
rospy.on_shutdown(stop)  

		
if __name__ == '__main__':
    try:
        rospy.init_node('script', anonymous=True)
        sub = rospy.Subscriber('/scan', LaserScan, callback)
        controller()
    except rospy.ROSInterruptException:
		pass
