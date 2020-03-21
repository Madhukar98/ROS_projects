#!/usr/bin/env python
import message_filters
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
 
'''
depth = 1.1
setpoint = 1
 
Kp=10
Kd=10 '''
 
prev_error=0
signalstable0 = 1550
signalstable3 = 1600
signalstable4 = 1600
signalstable5 = 1600
 
def callback(Kp,Kd,Setpoint_depth,PWM1):
	#global PWM1, PWM2, PWM3, PWM4, PWM5, PWM6
	global error, prev_error, derivative, correction
	global depth, setpoint, kp, kd
	global signalstable0, signalstable3, signalstable4, signalstable5
	
	error = Setpoint_depth - PWM1 
	derivative = error - prev_error 
	correction = Kp*error + Kd*derivative
	prev_error = error 
 
	PWM1 = signalstable0
	PWM2 = signalstable0
	PWM3 = signalstable3 + (correction*300)
	PWM4 = signalstable4 + (correction*300) 
	PWM5 = signalstable5 + (correction*300)
	PWM6 = 1500
	
	print(PWM1, PWM2, PWM3, PWM4, PWM5, PWM6)




def calculate_pid():
	
 
	rospy.init_node("depth_pid", anonymous = False)
	pub_pwm1=rospy.Publisher('PWM1', Int32, queue_size=10)
	pub_pwm2=rospy.Publisher('PWM2', Int32, queue_size=10)
	pub_pwm3=rospy.Publisher('PWM3', Int32, queue_size=10)
	pub_pwm4=rospy.Publisher('PWM4', Int32, queue_size=10)
	pub_pwm5=rospy.Publisher('PWM5', Int32, queue_size=10)
	pub_pwm6=rospy.Publisher('PWM6', Int32, queue_size=10)
	
	kp_value = message_filters.Subscriber('Kp', Kp)
	kd_value = message_filters.Subscriber('Kd', Kd)
	depth_value = message_filters.Subscriber('Setpoint_depth', depth)
	setpoint_value = message_filters.Subscriber('PWM1', setpoint)
	ts = message_filters.TimeSynchronizer([kp_value, kp_value, depth_value, setpoint_value], 10)
	ts.registerCallback(callback)
	#print("dsa")
	pub_pwm1.publish(PWM1)
	pub_pwm2.publish(PWM2)
	pub_pwm3.publish(PWM3)
	pub_pwm4.publish(PWM4)
	pub_pwm5.publish(PWM5)
	pub_pwm6.publish(PWM6)
	
	
 
if __name__ == "__main__":
	try:
		calculate_pid()
		rate = rospy.Rate(5)
		rospy.spin()
	except:
		pass
