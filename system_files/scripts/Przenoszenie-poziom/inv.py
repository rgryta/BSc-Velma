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

class MarkerPublisherThread:
    def threaded_function(self, obj):
        pub = MarkerPublisher("attached_objects")
        while not self.stop_thread:
            pub.publishSinglePointMarker(PyKDL.Vector(), 1, r=1, g=0, b=0, a=1, namespace='default', frame_id=obj.link_name, m_type=Marker.CYLINDER, scale=Vector3(0.02, 0.02, 1.0), T=pm.fromMsg(obj.object.primitive_poses[0]))
            try:
                rospy.sleep(0.1)
            except:
                break

        try:
            pub.eraseMarkers(0, 10, namespace='default')
            rospy.sleep(0.5)
        except:
            pass

    def __init__(self, obj):
        self.thread = Thread(target = self.threaded_function, args = (obj, ))

    def start(self):
        self.stop_thread = False
        self.thread.start()

    def stop(self):
        self.stop_thread = True
        self.thread.join()


 # define a function for frequently used routine in this test
def planAndExecute(velma, q_dest):
    # type: (object, object, object) -> object
    goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(10):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        #print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15,
                          planner_id="RRTConnect")
        if traj == None:
            continue
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

def moveInCartImpMode(velma, T_B_dest):
    if not velma.moveCartImpRight([T_B_dest], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
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

def rotateTorso(velma, torso_angle, q_map):
    q_map['torso_0_joint'] = torso_angle
    velma.moveJoint(q_map, 4.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(10)

def grabWithRightHand(velma):
    dest_q = [76.0/180.0*math.pi, 76.0/180.0*math.pi, 76.0/180.0*math.pi, 0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 650, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(10)
    rospy.sleep(0.5)
    if isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        print "Couldn't grab"
        exitError(11)
    rospy.sleep(0.5)

def moveToPositionZero(velma):
    velma.moveJointImpToCurrentPos(start_time=0.5)
    velma.waitForJoint()
    planAndExecute(velma, q_map_starting)

def findCanOnTable(table_tf, cafe_tf, can_tf):
    [t0_x, t0_y, t0_z] = table_tf.p
    [t1_x, t1_y, t1_z] = cafe_tf.p
    [c_x, c_y, c_z] = can_tf.p

    can_to_t0 = (c_x - t0_x)**2 + (c_y - t0_y)**2 + (c_z - t0_z)**2
    can_to_t1 = (c_x - t1_x)**2 + (c_y - t1_y)**2 + (c_z - t1_z)**2

    return "table" if can_to_t0 < can_to_t1 else "cafe"

def switchToJntMode(velma):
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)
 
    rospy.sleep(0.5)
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

    rospy.sleep(0.5) 
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

def adjustCornerPos(x, y, cx, cy, angle):
    x1 = x*math.cos(angle) - y*math.sin(angle) + cx
    y1 = x*math.sin(angle) + y*math.cos(angle) + cy
    return [x1, y1]

def getCorners(tPos, width, length, angle):
    cx = tPos[0]
    cy = tPos[1]
    x1 = width/2
    y1 = length/2
    c1 = adjustCornerPos(x1, y1, cx, cy, angle)
    x1 = width/2
    y1 = -length/2
    c2 = adjustCornerPos(x1, y1, cx, cy, angle)
    x1 = -width/2
    y1 = -length/2
    c3 = adjustCornerPos(x1, y1, cx, cy, angle)
    x1 = -width/2
    y1 = length/2
    c4 = adjustCornerPos(x1, y1, cx, cy, angle)
    #c1>c2>c3>c4>c1
    return [c1, c2, c3, c4]

def getClosestPointToLine(segPos, wrPos):
    xDelta = segPos[1][0] - segPos[0][0]
    yDelta = segPos[1][1] - segPos[0][1]
    l = ((wrPos[0] - segPos[0][0]) * xDelta + (wrPos[1] - segPos[0][1]) * yDelta) / (xDelta * xDelta + yDelta * yDelta)
    if l <= 0:
        [xc, yc] = segPos[0]
    elif l >= 1:
        [xc, yc] = segPos[1]
    else:
        xc = segPos[0][0] + l*xDelta
        yc = segPos[0][1] + l*yDelta
    return [xc, yc]


def getDistance(x, y, xc, yc):
    dx = math.pow(x-xc,2)
    dy = math.pow(y-yc,2)
    return math.sqrt(dx+dy)

def getClosestPoint(wrPos, tFrame, width, length):
    x = tFrame.p[0]
    y = tFrame.p[1]
    angle=getAngleFromRot(tFrame.M,'y')
    xc = wrPos[0]
    yc = wrPos[1]
    [c1, c2, c3, c4] = getCorners([x, y], width, length, angle)
    finalX = -1
    finalY = -1
    finalD = -1
    segments = [[c1, c2],[c2, c3],[c3, c4],[c4, c1]]
    for segment in segments:
        [xf, yf] = getClosestPointToLine(segment, [xc, yc])
        dist = getDistance(xf, yf, xc, yc)
        if finalD == -1 or dist<finalD:
            finalX = xf
            finalY = yf
            finalD = dist
    return [finalX, finalY]

def printData(velma):
    global timestamp
    global state
    print "____________________"
    print rospy.get_time()-timestamp #czas od ostatniego checkpointu
    print math.pi/2-abs(velma.getTf("Wo", "Gr").M.GetRPY()[1]) #poziom chwytaka
    tempstate = velma.getLastJointState()[1]
    for key in q_map_starting:
        print abs(tempstate[key]-state[key])
    timestamp = rospy.get_time()
    state=tempstate

def begin():
    print "Execution Time"
    print "Gripper Level"
    for key in q_map_starting:
        print "Absolute value of "+key+" position difference"

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__

if __name__ == "__main__":
    # define some configurations
    blockPrint()
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_aq2 = {'torso_0_joint':0,
        'right_arm_0_joint':-0.5,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.6,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_aq = {'torso_0_joint':0,
        'right_arm_0_joint':-0.5,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.6,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.05,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':-2.8,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    #standard initialization
    rospy.init_node('thesis')
    rospy.sleep(0.5)

    #Initializing robot...
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

    #adding octomap to planner
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize Planner"
        exitError(2)
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)

    switchToJntMode(velma)

    #Moving to position zero
    moveToPositionZero(velma)


    #Rotating robot...
    # can position
    T_Wo_Can = velma.getTf("Wo", "beer")
    T_Wo_Table_0 = velma.getTf("Wo", "table")
    T_Wo_Table_1 = velma.getTf("Wo", "cafe")

    target_table = findCanOnTable(T_Wo_Table_0, T_Wo_Table_1, T_Wo_Can) # na ktorym stoliku znajduje sie puszka

    Can_x = T_Wo_Can.p[0]   
    Can_y = T_Wo_Can.p[1]
    Can_z = T_Wo_Can.p[2]

    torso_angle = normalizeTorsoAngle(math.atan2(Can_y, Can_x))
    rotateTorso(velma, torso_angle, q_map_aq)
    
    enablePrint()
    print "____________________\n/START"
    begin()
    timestamp = rospy.get_time()
    state = velma.getLastJointState()[1]

    switchToCartMode(velma)
    moveForEquilibrium(velma)

    #moving grip halfway to can
    pos1 = velma.getTf("Wo", "Gr")
    pos2 = velma.getTf("Wo", "beer")
    vector = pos2.p - pos1.p
    xAngle = math.atan2(vector[1],vector[0])
    #move_rotation = PyKDL.Rotation.RPY(-xAngle,math.pi/2, 0) #1st rotation
    move_rotation = PyKDL.Rotation.RPY(math.pi+xAngle, -math.pi/2, 0) #2nd rotation

    move_vector = getAdjCanPos(pos1.p,T_Wo_Can.p, 0.3)+PyKDL.Vector(0, 0, T_Wo_Can.p[2]+0.01)
    to_can_frame = PyKDL.Frame(move_rotation, move_vector)
    moveInCartImpMode(velma, to_can_frame)
    printData(velma) #checkpoint

    arm_state = to_can_frame #save returning point for after grab

    move_vector = getAdjCanPos(pos1.p,T_Wo_Can.p, 0.012)+PyKDL.Vector(0, 0, T_Wo_Can.p[2]+0.01)
    to_can_frame = PyKDL.Frame(move_rotation, move_vector)
    moveInCartImpMode(velma, to_can_frame)
    printData(velma) #checkpoint

    #Grabbing the can
    grabWithRightHand(velma)
    printData(velma) #checkpoint

    #Moving right gripper back up
    arm_frame = PyKDL.Frame(move_rotation,arm_state.p+PyKDL.Vector(0, 0, 0.1))
    moveInCartImpMode(velma, arm_frame)
    printData(velma) #checkpoint

    #Switching to jnt_mode
    switchToJntMode(velma)
    if target_table == "table":
        target_table = "cafe"
        #go to: cafe
    else:
        target_table = "table"
        #go to: table
    T_Wo_Dest = velma.getTf("Wo", target_table)
    Target_x = T_Wo_Dest.p[0]
    Target_y = T_Wo_Dest.p[1]

    torso_angle = normalizeTorsoAngle(math.atan2(Target_y, Target_x))

    stateUpdate = velma.getLastJointState()[1] #(genpy.Time, {lastState})
    rotateTorso(velma, torso_angle, stateUpdate)
    printData(velma) #checkpoint

    #Move to target table
    switchToCartMode(velma)
    T_Wo_table = velma.getTf("Wo", target_table)    #calculating position for can placement
    Wr_pos=velma.getTf("Wo", "Gr") #save reference frame
    [xf, yf] = getClosestPoint(Wr_pos.p,T_Wo_table,1.3,0.6)
    table_height=1.2
    zf = T_Wo_table.p[2]+table_height

    move_rotation=velma.getTf("B", "Gr") #save reference frame

    moveForEquilibrium(velma)

    #Start gripper move
    place_can_frame = PyKDL.Frame(move_rotation.M, PyKDL.Vector(Wr_pos.p[0], Wr_pos.p[1], zf+0.05))
    place_can_frame_up = PyKDL.Frame(move_rotation.M, PyKDL.Vector(Wr_pos.p[0], Wr_pos.p[1], zf-0.03))
    moveInCartImpMode(velma, place_can_frame)
    printData(velma) #checkpoint
    place_can_frame = PyKDL.Frame(move_rotation.M, PyKDL.Vector(xf, yf, zf+0.05))
    moveInCartImpMode(velma, place_can_frame)
    printData(velma) #checkpoint
    place_can_frame = PyKDL.Frame(move_rotation.M, PyKDL.Vector(xf, yf, zf-0.03))
    moveInCartImpMode(velma, place_can_frame)
    printData(velma) #checkpoint

    #Release object
    openRightHand(velma)
    printData(velma) #checkpoint

    #Gripper move back
    moveInCartImpMode(velma, place_can_frame_up)
    printData(velma) #checkpoint


    #Return to start position
    switchToJntMode(velma)
    moveToPositionZero(velma)
    #printData(velma) #checkpoint
print "/END"
