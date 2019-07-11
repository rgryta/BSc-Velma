#!/usr/bin/env python

## Runs test for simple head motions.
# @ingroup integration_tests
# @file test_head.py
# @namespace scripts.test_head Integration test

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import copy

from velma_common import *
from planner import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    # define some configurations
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    rospy.init_node('head_test', anonymous=True)

    rospy.sleep(0.5)


    velma = VelmaInterface()
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    if velma.enableMotors() != 0:
        exitError(2)

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(3)

    print "waiting for Planner init..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(4)
    print "Planner init ok"

    # define a function for frequently used routine in this test
    def planAndExecute(q_dest):
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        for i in range(5):
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if traj == None:
                continue
            print "Executing trajectory..."
            if not velma.moveJointTraj(traj, start_time=0.5):
                exitError(5)
            if velma.waitForJoint() == 0:
                break
            else:
                print "The trajectory could not be completed, retrying..."
                continue
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        if not isConfigurationClose(q_dest, js[1]):
            exitError(6)

    def move_head(x,y):
	    q_dest = (x,y)
	    velma.moveHead(q_dest, 2.0, start_time=0.5)
	    if velma.waitForHead() != 0:
		exitError(4)
	    rospy.sleep(0.5)
	    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
		exitError(5)
            print "OK"

    def move_body(angle):
        q_map = {'torso_0_joint':angle,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }
	planAndExecute(q_map)



    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(7)

    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(8)

 
    planAndExecute(q_map_starting)

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
        print "This test requires starting pose:"
        print q_map_starting
        exitError(9)

    print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    move_head(0,0)
    move_head(0, -0.99) # up
    move_head(0, 0) # 
    move_body(0.2)
    move_body(0.4)
    move_body(0.6)
    move_body(0.8)
    move_body(1.0)
    move_body(1.2)
    move_head(0, -0.99) #  up
    move_head(0, 0) #

    move_head(1.56, -0.99) # up left
    move_head(1.56, 0) #


    


    move_head(1.56, 1.29) # down left
    move_head(1.56, 0) # zero left

    move_head(0, 1.29) # down left
    move_head(0, 0) # zero left
    move_body(1.0)
    move_body(0.8)
    move_body(0.6)
    move_head(0, 1.29) # down left
    move_head(0, 0) # zero left
    move_body(0.4)
    move_body(0.2)
    move_body(0.0)
    move_head(0, 1.29) # down left
    move_head(0, 0) # zero left





    move_head(0, -0.99) # right up
    move_head(0, 0) # right
    move_body(-0.2)
    move_body(-0.4)
    move_body(-0.6)
    move_body(-0.8)
    move_body(-1.0)
    move_body(-1.2)
    move_head(0, -0.99) # right up
    move_head(0, 0) # right

    move_head(-1.56, -0.99) # right up
    move_head(-1.56, 0) # right




    
    move_head(-1.56, 1.29) # right down
    move_head(-1.56, 0) # right zero

    move_head(0, 1.29) # down left
    move_head(0, 0) # zero left
    move_body(-1.0)
    move_body(-0.8)
    move_body(-0.6)
    move_head(0, 1.29) # down left
    move_head(0, 0) # zero left
    move_body(-0.4)
    move_body(-0.2)
    move_body(0.0)
    move_head(0,0) # zero zero

    exitError(0)



