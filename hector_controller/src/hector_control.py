#!/usr/bin/env python

import roslib; roslib.load_manifest('hector_controller')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# must be of type nav_msg/Odometry
# pose->pose->position->x,y; pitch is y, theta; roll is x, phi; linear
global_pose_topic = '/ground_truth/state'

# must be of type geometry_msgs/Twist; linear xyz, angular xyz
cmd_topic = '/cmd_vel'

# three dimensional integral measurements
integral = [0.0,0.0,0.0]

# three dimensional error in previous pass
previous_error = [0.0,0.0,0.0]

# default values for gains, goal (x,y,z), and delay (seconds)
rospy.set_param('gains', {'p': 0.85, 'i': 0.000, 'd': 0.35})
rospy.set_param('goal', [0.0,0.0,2.0])
rospy.set_param('delay', 0.001)

pub = rospy.Publisher(cmd_topic, Twist)

def odometry_callback(odom_data):
	pose = [odom_data.pose.pose.position.x,odom_data.pose.pose.position.y,odom_data.pose.pose.position.z]

        # rospy.wait_for_message does not work after rospy.spin is started, so use a param to communicate pose to the waypoint control
        rospy.set_param('pose', pose)
        gains = rospy.get_param('gains')
        tau_p, tau_i, tau_d = gains['p'], gains['i'], gains['d']
        goal = rospy.get_param('goal')
        delay = rospy.get_param('delay')
        global previous_error, integral

        
        ## calculate line between current pose and goal

        ## calculate difference between current bearing and slope of goal line

        ## pitch (theta) :: distance to goal
        ## yaw (psi) :: bearing deviation
        ## z :: as-is

        # initialize previous_error for first pass
	for i in range(3):
		if (previous_error[i] == 0.0):
			previous_error[i] = goal[i] - pose[i]

        # calculate error as :=goal-pose
        error = map(lambda a,b:a-b, goal, pose)

        # added error to establised integral value
	integral =  map(lambda a,b:a+b, integral, error)

        # calculate derivative as :=error-previous_error
 	derivative = map(lambda a,b:a-b, error, previous_error)

        # set previous_error for next pass
	previous_error = error

	[phi, theta, zed] = [(tau_p * error[i]) + (tau_d * derivative[i]) + (tau_i * integral[i]) for i in range(3)]
	twist = Twist()
	twist.linear.x=phi
	twist.linear.y=theta
	twist.linear.z=zed
#        rospy.loginfo("tau_p = " + str(tau_p) + "; x = " + str(odom_data.pose.pose.position.x)[:5] + "; e = " + str(error[0])[:5] + "; d = " + str(derivative[0])[:5] + "; phi = " + str(phi)[:5])
	pub.publish(twist)
	rospy.sleep(delay)
	
if __name__ == '__main__':
	try:
        	rospy.init_node('quadrotor_control', anonymous=True)
        	rospy.Subscriber(global_pose_topic, Odometry, odometry_callback, queue_size=2)
        	rospy.spin()
	except rospy.ROSInterruptException: pass

