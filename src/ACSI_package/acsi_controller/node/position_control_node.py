#!/usr/bin/env python2

#TODO: May need to port to python3 but currently using tf which is python 2 dependent
#TODO: Look up issue with pitch flipping
#TODO: Yaw_rate may be absolute yaw? needs better investigation

'''
Euler angle order needs to be ZXY for consistent stability 
Follows Right hand convention. X is forward Z is right Y is up in drone frame
TODO: Possible issues with Optitrack environment alignment on initialization. May need a calibration protocol
'''

import rospy
from acsi_controller.msg import Attitude_Setpoint, Attitude_Error
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseStamped, Quaternion
from std_msgs.msg import String
import numpy as np
import tf
from math import sin, cos, pi

start_time = 0
recieved_trajectory = False
recieved_optitrack = False
current_pose = Pose()
trajectory = PoseArray()

#TODO: May need to check axis convention to align with optitrack

def five_point_stencil(error,Ts):
    
    derivitive = Attitude_Error()
    try:
        derivitive.x = -(-error[0].x + 8.0*error[1].x - 8.0*error[3].x + error[4].x)/(12.0*Ts)
        derivitive.y = -(-error[0].y + 8.0*error[1].y - 8.0*error[3].y + error[4].y)/(12.0*Ts)
        derivitive.z = -(-error[0].z + 8.0*error[1].z - 8.0*error[3].z + error[4].z)/(12.0*Ts)
        derivitive.yaw = -(-error[0].yaw + 8*error[1].yaw - 8*error[3].yaw + error[4].yaw)/(12.0*Ts)
    except:
        print('division by zero')

    #print(derivitive)

    return derivitive

def yaw_difference(current_quaternion,desired_quaternion):
    current_explicit_quat = [current_quaternion.x, current_quaternion.y, current_quaternion.z, current_quaternion.w]
    desired_explicit_quat = [desired_quaternion.x, desired_quaternion.y, desired_quaternion.z, desired_quaternion.w]

    quat_diff =tf.transformations.quaternion_multiply(desired_explicit_quat,tf.transformations.quaternion_conjugate(current_explicit_quat)) #Maybe ypr convention is wrong
    euler_diff = tf.transformations.euler_from_quaternion(quat_diff)

    return euler_diff[2]

def error_update(current_pose,desired_pose,error_hist):
    
    derivitive = Attitude_Error()

    if len(error_hist) < 10:
        error_hist.insert(0,Attitude_Error())
        error_calc(current_pose,desired_pose,error_hist)
    else:
        error_hist.insert(0,Attitude_Error())
        error_calc(current_pose,desired_pose,error_hist)
        error_hist.pop()
    
    if len(error_hist) == 2:
        error_calc(current_pose,desired_pose,error_hist)

    
    if len(error_hist) >= 10:
        derivitive = five_point_stencil(error_hist,1.0/100.0)

    return error_hist, derivitive

def error_calc(current_pose,desired_pose,error_hist):

    yaw = get_yaw(current_pose.orientation)
    
    global_err = Attitude_Error()

    global_err.x = desired_pose.position.x - current_pose.position.x
    global_err.y = desired_pose.position.y - current_pose.position.y
    global_err.z = desired_pose.position.z - current_pose.position.z
    global_err.yaw = yaw_difference(current_pose.orientation,desired_pose.orientation)

    rel_err = Attitude_Error()

    rel_err.yaw = global_err.yaw
    rel_err.y = global_err.y

    np_err = yaw_transform(global_err,yaw)
    rel_err.x = np_err[0][0]
    rel_err.z = np_err[1][0]
    error_hist[0] = rel_err

def spin_controller(current_pose,desired_pose,error_hist,integral): #

    error_hist, derivitive = error_update(current_pose,desired_pose,error_hist)
    calculated_setpoint = Attitude_Setpoint()

    calculated_setpoint.thrust, integral = altitude_controller(error_hist,integral,derivitive)
    calculated_setpoint.yaw_rate, integral =  yaw_controller(error_hist,integral,derivitive)

    calculated_setpoint.pitch, calculated_setpoint.roll, integral = position_controller(error_hist,integral,derivitive)
    #print(derivitive)
    return calculated_setpoint

def altitude_controller(error, integral,derivitive):#\

    thrust = 47000 + altitude_proportional(error) - altitude_derivitive(error,derivitive)
    if thrust >= 65535:
        thrust = 65534
    if thrust < 0:
        thrust = 0
    
    return thrust, integral

def altitude_proportional(error):#
    p = 25000 # original 1/1.5
    return p * error[0].y

def altitude_integral(error ,integral):#
    i = 25000/2/100
    integral.y = integral.y + error[0].y*i
    if(integral.y > 5000):
        integral.y = 5000
    elif(integral.y < -5000):
        integral.y = -5000
    return integral
def altitude_derivitive(error ,derivitive):#
    d = 25000*1
    return d * derivitive.y

def yaw_controller(error, integral ,derivitive):#

    yaw_rate = 0

    return yaw_rate, integral

def yaw_proportional(error):#
    p = 0
    return p * error[0].yaw_rate

def yaw_integral(error,integral):#
    i = 0

def yaw_derivitive(error,derivitive):# Need to check what the yaw command actually is, rate or absolute. I'm leaning absolute
    d = 0
    return d * derivitive.yaw

def yaw_transform(global_position,yaw):

    rot = yaw
    rot_matrix = np.array([[cos(rot), -sin(rot)],[sin(rot), cos(rot)]])
    position_vector = np.array([[global_position.x],[global_position.z]])
    rel_position = np.dot(rot_matrix,position_vector)
    return rel_position

def get_yaw(orientation):
    explicit_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    current_euler = tf.transformations.euler_from_quaternion(explicit_quat,'szxy')
    return current_euler[2]

def get_euler(orientation):
    explicit_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    current_euler = tf.transformations.euler_from_quaternion(explicit_quat, "szxy")
    return current_euler

def position_controller(error,integral,derivitive):#
    

    pitch_set = pitch_proportional(error) - pitch_derivitive(error,derivitive)
    if pitch_set > 25:
        pitch_set = 25
    if pitch_set < -25:
        pitch_set = -25

    roll_set = roll_proportional(error) -  roll_derivitive(error,derivitive)

    if roll_set > 25:
        roll_set = 25
    if roll_set < -25:
        roll_set = -25


    return pitch_set, -roll_set, integral

def pitch_proportional(error):# needs sat
    p = .24 * 180 / pi

    #sat
    print(error[0].z)
    return p * error[0].z

def pitch_integral(error,integral):#
    i = .005 * 180 / pi
    integral.z = integral.z + error[0].z*i
    if(integral.z > 10):
        integral.z = 10
    elif(integral.z < -10):
        integral.z = -10
    print(integral.z)
    return integral

def pitch_derivitive(error,derivitive):#
    d = .24 * 180 / pi
    return d * derivitive.z

def roll_proportional(error):# needs sat
    p = .20 * 180 / pi
    #sat
    print(error[0].x)
    return p * error[0].x

def roll_integral(error,integral):#
    i = .005 * 180 / pi
    integral.x = integral.x + error[0].z*i

    if(integral.x > 10):
        integral.x = 10
    elif(integral.x < -10):
        integral.x = -10
    print(integral.x)
    return integral
def roll_derivitive(error,derivitive):#
    d = .18 * 180 / pi
    return d * derivitive.x

def optitrack_callback(opti_message):#
    global current_pose, recieved_optitrack

    current_pose = opti_message.pose
    recieved_optitrack = True

def trajectory_callback(trajectory_in):#
    global recieved_trajectory, trajectory, start_time

    if recieved_trajectory == False and len(trajectory_in.poses) > 2 and rospy.Time.now().secs-start_time > 15:
        trajectory = trajectory_in
        recieved_trajectory = True
    
if __name__ == '__main__':



    rospy.init_node('position_controller_node')
    start_time = rospy.Time.now().secs
    status_pub = rospy.Publisher('pid_controller/status',String,queue_size=2)
    
    setpoint_pub = rospy.Publisher('controller/ypr',Attitude_Setpoint,queue_size=2)
    rospy.Subscriber('/vrpn_client_node/Crazyflie/pose',PoseStamped,optitrack_callback)
    rospy.Subscriber('/trajectory/drone_trajectory',PoseArray,trajectory_callback)
    error = [Attitude_Error()]
    integral = Attitude_Error()

    hover_pose = Pose()
    hover_pose.position.x = -.7
    hover_pose.position.y = 2
    hover_pose.position.z = .5
    hover_pose.orientation.x = 0
    hover_pose.orientation.y = 0
    hover_pose.orientation.z = 0
    hover_pose.orientation.w = 1

    land_pose = Pose()
    land_pose.position.x = -.7
    land_pose.position.y = 2
    land_pose.position.z = .5
    land_pose.orientation.x = 0
    land_pose.orientation.y = 0
    land_pose.orientation.z = 0
    land_pose.orientation.w = 1

    desired_pose = hover_pose

    r = rospy.Rate(100)
    start_time = rospy.Time.now().secs
    sequence = 0
    sequence2 = 0
    sequence3 = 0
    sequence4 = 0

    while not rospy.is_shutdown():

        current_setpoint = spin_controller(current_pose,desired_pose,error,integral)
        if(recieved_trajectory == True and recieved_optitrack == True and sequence < len(trajectory.poses) and rospy.Time.now().secs-start_time > 5):
            print('Trajectory Started')
            desired_pose = trajectory.poses[sequence]
            sequence = sequence + 1
        elif(recieved_trajectory == True and recieved_optitrack == True and sequence >= len(trajectory.poses) and sequence2 < 200):
            print('Holding')
            desired_pose = trajectory.poses[-1]
            sequence2 = sequence2 + 1
        elif(sequence >= len(trajectory.poses) and sequence2 >= 200 and sequence3 < 500):
            desired_pose = trajectory.poses[-1]
            desired_pose.position.y = 2.5
            sequence3 = sequence3 + 1
        elif rospy.Time.now().secs-start_time > 25:
            print('landing')
            current_setpoint.thrust = 0
        elif(recieved_trajectory == True and recieved_optitrack == True and sequence < len(trajectory.poses) and rospy.Time.now().secs-start_time < 10):
            print('Trajectory recieved, waiting to begin')
            desired_pose = hover_pose
        elif(recieved_trajectory == True and recieved_optitrack == False):
            print('Waiting for optitrak lock')
            desired_pose = hover_pose    
        elif(recieved_trajectory == False and recieved_optitrack == True):
            print('Waiting for Trajectory')
            desired_pose = hover_pose


        else:
            print('hovering')
            desired_pose = hover_pose

            

        setpoint_pub.publish(current_setpoint)

        r.sleep()

