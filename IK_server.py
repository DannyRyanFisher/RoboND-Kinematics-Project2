#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
# All Rights Reserved.
# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import *
import pickle
from Forward_Kinematics import Forward_Kinematics


# IK handler service
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### ADDED FK code here

        # Load pre-formed forward kinematic formulas
        pickle_in = open("FK.pickle", "rb")
        FK = pickle.load(pickle_in)


        ### ADDED IK code here

        # Find end-effector rotation matrix RPY (Roll, Pitch, Yaw)
        r,p,y = symbols('r p y')

        # Roll
        ROT_x = Matrix([[       1,       0,       0],
                        [       0,  cos(r), -sin(r)],
                        [       0,  sin(r),  cos(r)]])
        # Pitch
        ROT_y = Matrix([[  cos(p),       0,  sin(p)],
                        [       0,       1,       0],
                        [ -sin(p),       0,  cos(p)]])
        # Yaw
        ROT_z = Matrix([[  cos(y), -sin(y),       0],
                        [  sin(y),  cos(y),       0],
                        [       0,       0,       1]])

        ROT_EE = ROT_z * ROT_y * ROT_x

        # Compensate for rotation discrepancy between DH parameters and Gazebo
        # Correction Needed to Account for Orientation Difference Between
        # Definition of Gripper Link_G in URDF versus DH Convention
        ROT_corr = ROT_z.subs(y, 3.1415926535897931) * ROT_y.subs(p, -1.5707963267948966)
        ROT_EE = ROT_EE * ROT_corr

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # store EE position in a matrix
            EE = Matrix([[px],
                        [py],
                        [pz]])

            # roll, pitch, yaw = end-effector orientation
            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x,
                 req.poses[x].orientation.y,
                 req.poses[x].orientation.z,
                 req.poses[x].orientation.w])

            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            # Calculate Wrist Center
            WC = EE - (0.303) * ROT_EE[:,2]

            # Calculate joint angles using Geometric IK method
            # Calculate theta 1
            theta1 = atan2(WC[1],WC[0])

            # find the 3rd side of the triangle
            B = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))

            #Cosine Laws SSS to find all inner angles of the triangle
            a = acos((-0.6875 + B*B) / (2.5*B))
            b = acos(( 3.8125 - B*B) / (3.75))
            c = acos(( 0.6875 + B*B) / (3.0*B))

            #Find theta2 and theta3
            theta2 = pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
            theta3 = pi/2 - (b+0.036) # 0.036 accounts for sag in link4 of -0.054m

            # Extract rotation matrices from the transformation matrices
            # Extract rotation matrix R0_3 from transformation matrix T0_3 then substitute angles q1-3
            R0_3 = FK.T0_1[0:3,0:3] * FK.T1_2[0:3,0:3] * FK.T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={FK.q1: theta1, FK.q2: theta2, FK.q3: theta3})


            # Get rotation matrix R3_6 from (transpose of R0_3 * R_EE)
            R3_6 = R0_3.transpose() * ROT_EE

            # Euler angles from rotation matrix
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])

            # choose theta 4 and 6 (CAST)
            if (theta5 > pi) :
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1],-R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1],R3_6[1,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
