#!/usr/bin/env python2
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import copy
import time
import sys, os
import numpy as np

from velma_common import *
from rcprg_planner import *
import PyKDL
from threading import Thread

from rcprg_ros_utils import MarkerPublisher, exitError

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import tf_conversions.posemath as pm

def moveInCartImpMode(velma, T_B_dest, scaler):
    if not velma.moveCartImpRight([T_B_dest], [scaler], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    rospy.sleep(0.5)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

def moveForEquilibrium(velma):
    arm_frame = velma.getTf("B", "Gr")
    T_Wr_Gr = velma.getTf("Wr", "Gr")
    if not velma.moveCartImpRight([arm_frame], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(18)
    rospy.sleep(0.5)
    if velma.waitForEffectorRight() != 0:
        exitError(19)
    rospy.sleep(0.5)

def resetLeftArm(q_map):
    for key in q_map_left:
        q_map[key]=q_map_left[key]
    return q_map

def moveToQmap(velma,q_map, scaler):
    tempstate = q_map_starting
    for key in tempstate:
        tempstate[key]=q_map[key]
    tempstate=resetLeftArm(tempstate)
    rospy.sleep(0.5)
    velma.moveJoint(tempstate, scaler, start_time=0.5, position_tol=15.0/180.0*math.pi)
    rospy.sleep(0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(10)

def rotateTorso(velma, torso_angle, q_map, scaler):
    q_map['torso_0_joint'] = torso_angle
    moveToQmap(velma,q_map,scaler)
    

def grabWithRightHand(velma):
    dest_q = [76.0/180.0*math.pi, 76.0/180.0*math.pi, 76.0/180.0*math.pi, 0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        print "Couldn't grab"
        exitError(11)
    rospy.sleep(0.5)

def switchToJntMode(velma):
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)
 
    rospy.sleep(1)
    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not" 
        exitError(3)

def switchToCartMode(velma):
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        print "Cannot moveCartImpRightCurrentPos"
        exitError(9)

    if velma.waitForEffectorRight() != 0:
        print "waitForEffectorright error"
        exitError(8)

    if not velma.moveCartImpLeftCurrentPos(start_time=0.2):
        print "Cannot moveCartImpLeftCurrentPos"
        exitError(9)

    if velma.waitForEffectorLeft() != 0:
        print "waitForEffectorLeft error"
        exitError(8)

    rospy.sleep(1) 
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)

def openRightHand(velma):
    dest_q = [0, 0, 0, 0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(1)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(11)

def normalizeTorsoAngle(torso_angle):
    if torso_angle>math.pi/2:
        return math.pi/2-0.1
    elif torso_angle<-math.pi/2:
        return -math.pi/2+0.1
    else:
        return torso_angle

def getAngleFromRot(rotation, rotAngle):
    if rotAngle.lower() in ['r']:
        return rotation.GetRPY()[0]
    elif rotAngle.lower() in ['p']:
        return rotation.GetRPY()[1]
    elif rotAngle.lower() in ['y']:
        return rotation.GetRPY()[2]

def getAdjCanPos(wrPos, canPos, r):
    #gets x,y Can position but moved r distance in favor of wrench position
    dx = wrPos[0]-canPos[0]
    dy = wrPos[1]-canPos[1]
    l = math.sqrt(math.pow(dx,2)+math.pow(dy,2))
    newPos = PyKDL.Vector(canPos[0]+(r/l)*dx,canPos[1]+(r/l)*dy,0)
    return newPos

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__

def printData(velma,deg=-math.pi):
    enablePrint()
    global timestamp
    global state
    print "____________________"
    print rospy.get_time()-timestamp #czas od ostatniego checkpointu
    if deg!=-1.0:
        print abs(deg-velma.getTf("Wo", "Gr").M.GetRPY()[0]) #poziom chwytaka
    else:
        print '0'
    tempstate = velma.getLastJointState()[1]
    for key in q_map_starting:
        print abs(tempstate[key]-state[key])
    timestamp = rospy.get_time()
    state=tempstate
    blockPrint()

def begin():
    print "Execution Time"
    print "Gripper Level"
    for key in q_map_starting:
        print "Absolute value of "+key+" position difference"

def init():
    velma = VelmaInterface()
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    
    if velma.enableMotors() != 0:
        exitError(14)

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)
    return velma

if __name__ == "__main__":
    # define some configurations
    #blockPrint()
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.8,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_left = {'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.8,
        'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85,
        'left_arm_4_joint':0,
        'left_arm_5_joint':0.5,
        'left_arm_6_joint':0 }


    #standard initialization
    rospy.init_node('thesis')
    rospy.sleep(0.5)

    #Initializing robot...
    velma=init()

    switchToJntMode(velma)

    #Moving to position zero
    moveToQmap(velma,q_map_starting,10.0)

    T_Wo_Handle = velma.getTf("Wo", "handle")
    Can_x = T_Wo_Handle.p[0]   
    Can_y = T_Wo_Handle.p[1]
    Can_z = T_Wo_Handle.p[2]

    torso_angle = normalizeTorsoAngle(math.atan2(Can_y, Can_x))
    rotateTorso(velma, torso_angle, q_map_starting,5.0)

    #Rotating robot...
    # can position

    enablePrint()
    print "____________________\n/START"
    begin()
    timestamp = rospy.get_time()
    state = velma.getLastJointState()[1]

    switchToCartMode(velma)
    moveForEquilibrium(velma)

    #moving grip halfway to can
    T_Wo_Grip = velma.getTf("Wo", "Gr")
    move_rotation = PyKDL.Rotation.RPY(0, math.pi/2, 0) #1st rotation


    move_vector = getAdjCanPos(T_Wo_Grip.p,T_Wo_Handle.p, 0.2)+PyKDL.Vector(0, 0, T_Wo_Handle.p[2])
    to_can_frame = PyKDL.Frame(move_rotation, move_vector)
    moveInCartImpMode(velma, to_can_frame, 15.0)
    printData(velma) #checkpoint

    #move_rotation = PyKDL.Rotation.RPY(math.pi, -math.pi/2, 0) #2nd rotation

    #move_vector = getAdjCanPos(T_Wo_Grip.p,T_Wo_Handle.p, 0.2)+PyKDL.Vector(0, 0, T_Wo_Handle.p[2])
    #to_can_frame = PyKDL.Frame(move_rotation, move_vector)
    #moveInCartImpMode(velma, to_can_frame, 15.0)
    #printData(velma) #checkpoint



    move_vector = getAdjCanPos(T_Wo_Grip.p,T_Wo_Handle.p, 0.0)+PyKDL.Vector(0, 0, T_Wo_Handle.p[2])




















print "/END"
