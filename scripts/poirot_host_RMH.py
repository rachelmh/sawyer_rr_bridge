#!/usr/bin/env python
import roslib
roslib.load_manifest('sawyer_rr_bridge')
import rospy
import intera_interface
from std_msgs.msg import Empty
from intera_core_msgs.msg import IOComponentCommand

import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from intera_core_msgs.msg import DigitalIOState

from sawyer_pykdl import sawyer_kinematics
from sensor_msgs.msg import Joy
sawyer_servicedef="""
#Service to provide simple interface to Sawyer
service SawyerJoint_Interface

option version 0.4

object Sawyer
property double[] joint_positions
property double[] joint_velocities
property double[] joint_torques
property double[] endeffector_positions
property double[] endeffector_orientations
property double[] endeffector_twists
property double[] endeffector_wrenches
property double[] endeffector_velocity
property double[] pseudoinverse_Jacobian
property double[] Jacobian
property double[] inertia
property double[] SpaceNavigatorJoy
property int32 CuffButton
property int32 OkWheelButton
property int32 XButton
function void setControlMode(uint8 mode)
function void setJointCommand(string limb, double[] command)
function void setDisplayImage(string img)
function void setPositionModeSpeed(double speed)
function void readJointPositions()
function void Sawyer_movetoNeutral()
function void setDIOCommand(string name, int32 cmd)





function double[] solveIKfast(double[] positions, double[] quaternions, string limb_choice)

end object

"""
class Sawyer_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('poirot_rrhost')
        
        print "Enabling Robot"
        rs = intera_interface.RobotEnable()
        rs.enable()
        self.rs1=intera_interface.RobotEnable()
        rospy.Subscriber("spacenav/joy",Joy, self.SpaceNavJoyCallback)
        rospy.Subscriber("/robot/digital_io/right_lower_cuff/state" , DigitalIOState, self.CuffCallback)
        rospy.Subscriber("/robot/digital_io/right_button_ok/state" , DigitalIOState, self.NavWheelCallback)
        rospy.Subscriber("/robot/digital_io/right_button_triangle/state", DigitalIOState,self.XCallback)
        self.DIOpub=rospy.Publisher('/io/comms/io/command', IOComponentCommand, queue_size=10)
        self.h=IOComponentCommand()
        self.h.time=rospy.Time.now()
        self.h.op="set"
        self.h.args= "{ \"signals\" : { \"port_sink_0\" : { \"format\" : {  \"type\" : \"int\",  \"dimensions\" : [ 1] }, \"data\" : [ 0] } } }" 




        self.nav = intera_interface.Navigator()
        #self.nav.register_callback(self.ok_pressed, '_'.join([nav_name, 'button_ok']))



        #intera_interface/robot/digital_io/right_lower_cuff/state
         
        # self._valid_limb_names = {'left': 'left', 
        #                             'l': 'left', 
        #                             'right': 'right',
        #                             'r': 'right'}
        self._valid_limb_names = {'right': 'right',
                                    'r': 'right'}
        
        # get information from the SDK
       # self._left = intera_interface.Limb('left')
        self._right = intera_interface.Limb('right')
        #self._l_jnames = self._left.joint_names()
        self._r_jnames = self._right.joint_names()
        self.kin=sawyer_kinematics('right')
        
        # data initializations
        self._jointpos = [0]*7
        self._jointvel = [0]*7
        self._jointtor = [0]*7
        self._ee_pos = [0]*3
        self._ee_or = [0]*4
        self._ee_tw = [0]*6
        self._ee_wr = [0]*6
        self._ee_vel = [0]*6

        self._pijac=[]  #numpy.matrix('0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0')
        self._jac=[]  #numpy.matrix('0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,0')
        self._inertia_mat=[] #numpy.matrix('0,0,0,0,0,0,0;0,0,0,0,0,0,0;0,0,0,0,0,0,0;0,0,0,0,0,0,0;0,0,0,0,0,0,0;0,0,0,0,0,0,0;0,0,0,0,0,0,0')

        self.Jxl_raw=0
        self.Jyl_raw=0
        self.Jzl_raw=0
        self.Jxa_raw=0
        self.Jya_raw=0
        self.Jza_raw=0
        self.leftbutton=0
        self.rightbutton=0
        self.spacenav=[]


        #self._l_joint_command = dict(zip(self._l_jnames,[0.0]*7))
        self._r_joint_command = dict(zip(self._r_jnames,[0.0]*7))
        self.MODE_POSITION = 0;
        self.MODE_VELOCITY = 1;
        self.MODE_TORQUE = 2;
        self._mode = self.MODE_POSITION
        self.RMH_delay=.01
        # initial joint command is current pose
        self.readJointPositions()
        #self.setJointCommand('left',self._jointpos[0:7])
        self.setJointCommand('right',self._jointpos[0:7])
        
        self.head_display = intera_interface.HeadDisplay()
        print('head display!')

        self.image="/home/rachel/ros_ws/src/sawyer_simulator/sawyer_sim_examples/models/cafe_table/materials/textures/Wood_Floor_Dark.jpg"
        self.head_display.display_image(self.image,display_rate=100)
        print('finished first head display')
        # Start background threads
        self._running = True
        self._t_joints = threading.Thread(target=self.jointspace_worker)
        self._t_joints.daemon = True
        self._t_joints.start()
        
        self._t_effector = threading.Thread(target=self.endeffector_worker)
        self._t_effector.daemon = True
        self._t_effector.start()
        
        self._t_command = threading.Thread(target=self.command_worker)
        self._t_command.daemon = True
        self._t_command.start()

        self._t_display = threading.Thread(target=self.display_worker)
        self._t_display.daemon = True
        self._t_display.start()

        self._t_dio = threading.Thread(target=self.dio_worker)
        self._t_dio.daemon = True
        self._t_dio.start()



    def close(self):
        self.h.args= "{ \"signals\" : { \"port_sink_0\" : { \"format\" : {  \"type\" : \"int\",  \"dimensions\" : [ 1] }, \"data\" : [ 0] } } }" 
        self.rs1.disable()
        self._running = False
        self._t_joints.join()
        self._t_effector.join()
        self._t_command.join()
        self._t_display.join()
        self._t_dio.join()
        
        if (self._mode != self.MODE_POSITION):
            
            self._right.exit_control_mode()

    @property	
    def joint_positions(self):
        return self._jointpos
    
    @property	
    def joint_velocities(self):
        return self._jointvel

    @property	
    def joint_torques(self):
        return self._jointtor
    
    @property 
    def endeffector_positions(self):
        return self._ee_pos
    
    @property 
    def endeffector_orientations(self):
        return self._ee_or
    
    @property 
    def endeffector_twists(self):
        return self._ee_tw
    
    @property 
    def endeffector_wrenches(self):
        return self._ee_wr
    
    @property 
    def endeffector_velocity(self):
        return self._ee_vel

    @property 
    def pseudoinverse_Jacobian(self):
        return self._pijac

    @property 
    def Jacobian(self):
        return self._jac

    @property 
    def inertia(self):
        return self._inertia_mat

    @property 
    def SpaceNavigatorJoy(self):
        return self.spacenav

    @property 
    def CuffButton(self):
        
        return self.cuffbutton

    @property
    def XButton(self):
        return self.xbutton
    

    @property 
    def OkWheelButton(self):
        return self.navbutton


    def readJointPositions(self):
        #l_angles = self._left.joint_angles()
        r_angles = self._right.joint_angles()
        #if l_angles:
        #    for i in xrange(0,len(self._l_jnames)):
        #        self._jointpos[i] = l_angles[self._l_jnames[i]]
        # if r_angles:
        #     for i in xrange(0,len(self._r_jnames)):
        #         self._jointpos[i+7] = r_angles[self._r_jnames[i]]
        if r_angles:
            for i in xrange(0,len(self._r_jnames)):
                self._jointpos[i] = r_angles[self._r_jnames[i]]
            
    def readJointVelocities(self):
        #l_velocities = self._left.joint_velocities()
        r_velocities = self._right.joint_velocities()
        #if l_velocities:
        #    for i in xrange(0,len(self._l_jnames)):
        #        self._jointvel[i] = l_velocities[self._l_jnames[i]]
        # if r_velocities:
        #     for i in xrange(0,len(self._r_jnames)):
        #         self._jointvel[i+7] = r_velocities[self._r_jnames[i]]
        if r_velocities:
            for i in xrange(0,len(self._r_jnames)):
                self._jointvel[i] = r_velocities[self._r_jnames[i]]
            
    def readJointTorques(self):
        #l_efforts = self._left.joint_efforts()
        r_efforts = self._right.joint_efforts()
        #if l_efforts:
        #    for i in xrange(0,len(self._l_jnames)):
        #        self._jointtor[i] = l_efforts[self._l_jnames[i]]
        if r_efforts:
            for i in xrange(0,len(self._r_jnames)):
                self._jointtor[i] = r_efforts[self._r_jnames[i]]
    
    def readEndEffectorPoses(self):
        # l_pose = self._left.endpoint_pose()
        # if l_pose:
        #     self._ee_pos[0] = l_pose['position'].x
        #     self._ee_pos[1] = l_pose['position'].y
        #     self._ee_pos[2] = l_pose['position'].z
        #     self._ee_or[0] = l_pose['orientation'].w
        #     self._ee_or[1] = l_pose['orientation'].x
        #     self._ee_or[2] = l_pose['orientation'].y
        #     self._ee_or[3] = l_pose['orientation'].z
        r_pose = self._right.endpoint_pose()
        if r_pose:
            self._ee_pos[0] = r_pose['position'].x
            self._ee_pos[1] = r_pose['position'].y
            self._ee_pos[2] = r_pose['position'].z
            self._ee_or[0] = r_pose['orientation'].w
            self._ee_or[1] = r_pose['orientation'].x
            self._ee_or[2] = r_pose['orientation'].y
            self._ee_or[3] = r_pose['orientation'].z

    def readKDL(self):
        temppij=self.kin.jacobian_pseudo_inverse()
        tempj=self.kin.jacobian()
        tempi=self.kin.inertia()

        self._pijac=numpy.array(temppij).flatten()
        self._jac=numpy.array(tempj).flatten()
        self._inertia_mat=numpy.array(tempi).flatten()
        
        
    def readEndEffectorTwists(self):
        # l_twist = self._left.endpoint_velocity()
        # if l_twist:
        #     self._ee_tw[0] = l_twist['angular'].x
        #     self._ee_tw[1] = l_twist['angular'].y
        #     self._ee_tw[2] = l_twist['angular'].z
        #     self._ee_tw[3] = l_twist['linear'].x
        #     self._ee_tw[4] = l_twist['linear'].y
        #     self._ee_tw[5] = l_twist['linear'].z
        r_twist = self._right.endpoint_velocity()
        if r_twist:
            self._ee_tw[0] = r_twist['angular'].x
            self._ee_tw[1] = r_twist['angular'].y
            self._ee_tw[2] = r_twist['angular'].z
            self._ee_tw[3] = r_twist['linear'].x
            self._ee_tw[4] = r_twist['linear'].y
            self._ee_tw[5] = r_twist['linear'].z

    def readEndEffectorWrenches(self):
        # l_wrench = self._left.endpoint_effort()
        # if l_wrench:
        #     self._ee_wr[0] = l_wrench['torque'].x
        #     self._ee_wr[1] = l_wrench['torque'].y
        #     self._ee_wr[2] = l_wrench['torque'].z
        #     self._ee_wr[3] = l_wrench['force'].x
        #     self._ee_wr[4] = l_wrench['force'].y
        #     self._ee_wr[5] = l_wrench['force'].z
        r_wrench = self._right.endpoint_effort()
        if r_wrench:
            self._ee_wr[0] = r_wrench['torque'].x
            self._ee_wr[1] = r_wrench['torque'].y
            self._ee_wr[2] = r_wrench['torque'].z
            self._ee_wr[3] = r_wrench['force'].x
            self._ee_wr[4] = r_wrench['force'].y
            self._ee_wr[5] = r_wrench['force'].z
    
    def readEndEffectorVelocity(self):

        r_ee_vel = self._right.endpoint_velocity()
        if r_pose:
            # self._ee_pos[0] = r_pose['position'].x
            # self._ee_pos[1] = r_pose['position'].y
            # self._ee_pos[2] = r_pose['position'].z
            # self._ee_or[0] = r_pose['orientation'].w
            # self._ee_or[1] = r_pose['orientation'].x
            # self._ee_or[2] = r_pose['orientation'].y
            # self._ee_or[3] = r_pose['orientation'].z
            self._ee_vel[0] = r_ee_vel['linear'].x
            self._ee_vel[1] = r_ee_vel['linear'].y
            self._ee_vel[2] = r_ee_vel['linear'].z
            self._ee_vel[3] = r_ee_vel['angular'].w
            self._ee_vel[4] = r_ee_vel['angular'].x
            self._ee_vel[5] = r_ee_vel['angular'].y

    def SpaceNavJoyCallback(self,data):
        #roslaunch spacenav_node classic.launch


        #I think these have to be self. variables due to the nature of callback. data is only accessed in the callback
        
        self.Jxl_raw=data.axes[0]
        self.Jyl_raw=data.axes[1]
        self.Jzl_raw=data.axes[2]

        self.Jxa_raw=data.axes[3]
        self.Jya_raw=data.axes[4]
        self.Jza_raw=data.axes[5]
        self.leftbutton=data.buttons[0]
        self.rightbutton=data.buttons[1]
        

        self.spacenav=[self.Jxl_raw,self.Jyl_raw,self.Jzl_raw,self.Jxa_raw,self.Jya_raw,self.Jza_raw, self.leftbutton,self.rightbutton]


    def CuffCallback(self,data):
        #roslaunch spacenav_node classic.launch


        #I think these have to be self. variables due to the nature of callback. data is only accessed in the callback
        
        self.cuffbutton=data.state
        #print(self.cuffbutton,type(self.cuffbutton))

    def NavWheelCallback(self,data):
        #roslaunch spacenav_node classic.launch


        #I think these have to be self. variables due to the nature of callback. data is only accessed in the callback
        
        self.navbutton=data.state
        #print(self.navbutton,type(self.navbutton))
    
    def XCallback(self,data):
        #roslaunch spacenav_node classic.launch


        #I think these have to be self. variables due to the nature of callback. data is only accessed in the callback
        
        self.xbutton=data.state

    def setControlMode(self, mode):
        if mode != self.MODE_POSITION and \
                mode != self.MODE_VELOCITY and \
                mode != self.MODE_TORQUE:
            return
        if mode == self.MODE_POSITION:
            # self._left.exit_control_mode()
            self._right.exit_control_mode()
            # set command to current joint positions
            # self.setJointCommand('left',self._jointpos[0:7])
            self.setJointCommand('right',self._jointpos[0:7])
        elif mode == self.MODE_VELOCITY:
            # set command to zeros
            # self.setJointCommand('left',[0]*7)
            self.setJointCommand('right',[0]*7)
        elif mode == self.MODE_TORQUE:
            # set command to zeros
            # self.setJointCommand('left',[0]*7)
            self.setJointCommand('right',[0]*7)
        
        self._mode = mode

    # This function calls RSDK ikFast Service
    def solveIKfast(self, positions, quaternions, limb_choice):
        ns = "ExternalTools/" + limb_choice + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {}
        # if (limb_choice == 'left' or limb_choice == 'l'):
        #     limb_choice = 'left'
        #     poses = {
        #         'left': PoseStamped(
        #             header=hdr,
        #             pose=Pose(
        #                 position = Point(
        #                     x = positions[0],
        #                     y = positions[1],
        #                     z = positions[2],
        #                 ),
        #                 orientation = Quaternion(
        #                     x = quaternions[1],
        #                     y = quaternions[2],
        #                     z = quaternions[3],
        #                     w = quaternions[0],
        #                 ),
        #             ),
        #         ),
        #         'right': PoseStamped(
        #             header=hdr,
        #             pose=Pose(
        #                 position = Point(
        #                     x = self._ee_pos[3],
        #                     y = self._ee_pos[4],
        #                     z = self._ee_pos[5],
        #                 ),
        #                 orientation = Quaternion(
        #                     x = self._ee_or[5],
        #                     y = self._ee_or[6],
        #                     z = self._ee_or[7],
        #                     w = self._ee_or[4],
        #                 ),
        #             ),
        #         ),
        #     }
        if (limb_choice == 'right' or limb_choice == 'r'):
            limb_choice = 'right'
            poses = {
                'left': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position = Point(
                            x = self._ee_pos[0],
                            y = self._ee_pos[1],
                            z = self._ee_pos[2],
                        ),
                        orientation = Quaternion(
                            x = self._ee_or[1],
                            y = self._ee_or[2],
                            z = self._ee_or[3],
                            w = self._ee_or[0],
                        ),
                    ),
                ),
                'right': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position = Point(
                            x = positions[0],
                            y = positions[1],
                            z = positions[2],
                        ),
                        orientation = Quaternion(
                            x = quaternions[1],
                            y = quaternions[2],
                            z = quaternions[3],
                            w = quaternions[0],
                        ),
                    ),
                ),
            }
        else:
            print "Not a valid arm"
            return 

        # begin the solvinng process
        ikreq.pose_stamp.append(poses[limb_choice])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)

        seed_dict = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }

        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = seed_dict.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "\nIK Joint Solution:\n", limb_joints
            print "------------------"
            print "Response Message:\n", resp
        # if no valid solution was found
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")

        return resp.joints[0].position


    def setJointCommand(self, limb, command):
        limb = limb.lower()
        if not limb in self._valid_limb_names.keys():
            return
        
        # if self._valid_limb_names[limb] == 'left':
        #     for i in xrange(0,len(self._l_jnames)):
        #         self._l_joint_command[self._l_jnames[i]] = command[i]
        if self._valid_limb_names[limb] == 'right':
            for i in xrange(0,len(self._r_jnames)):
                self._r_joint_command[self._r_jnames[i]] = command[i]

    def setDisplayImage(self,img):
        self.image=img
        #self.head_display.display_image(self.image)

    def setDIOCommand(self,name,cmd):
        self.h.time=rospy.Time.now()
        self.h.op="set"
        #h.args= "{ \"signals\" : { \"{name}\" : { \"format\" : {  \"type\" : \"int\",  \"dimensions\" : [ 1] }, \"data\" : [ {cmd} ] } } }"  
        self.h.args= "{ \"signals\" : { \"%s\" : { \"format\" : {  \"type\" : \"int\",  \"dimensions\" : [ 1] }, \"data\" : [ %d ] } } }"  %(name, cmd)
        #'{  time : now,  op : "set", args : "{ \"signals\" : { \"'$2'\" : { \"format\" : {  \"type\" : \"'$type'\",  \"dimensions\" : [ '$dimensions' ] }, \"data\" : [ '$3' ] } } }" }'
    
    def setPositionModeSpeed(self, speed):
        if speed < 0.0:
            speed = 0.0
        elif speed > 1.0:
            #speed = 1.0
            speed = 2.0
        
        # self._left.set_joint_position_speed(speed)
        self._right.set_joint_position_speed(speed)
        
    
    # worker function to request and update joint data for sawyer
    # maintain 100 Hz read rate
    # TODO: INCORPORATE USER-DEFINED JOINT PUBLISH RATE
    def jointspace_worker(self):
        while self._running:
            t1 = time.time()
            self.readJointPositions()
            self.readJointVelocities()
            self.readJointTorques()
            self.readKDL()

            while (time.time() - t1 < self.RMH_delay):
                # idle
                time.sleep(0.001)
                
            
    
    # worker function to request and update end effector data for sawyer
    # Try to maintain 100 Hz operation
    def endeffector_worker(self):
        while self._running:
            
            self.readEndEffectorPoses()
            self.readEndEffectorTwists()
            self.readEndEffectorWrenches()
            #self.head_display.display_image(self.image,display_rate=.001)
            t1 = time.time()

            while (time.time() - t1 < self.RMH_delay):
 
                time.sleep(0.001)
            
    # worker function to continuously issue commands to sawyer
    # Try to maintain 100 Hz operation
    # TODO: INCLUDE CLOCK JITTER CORRECTION
    def command_worker(self):
        while self._running:
            t1 = time.time()
            
            if (self._mode == self.MODE_POSITION):
                # self._left.set_joint_positions(self._l_joint_command)
                self._right.set_joint_positions(self._r_joint_command)
            elif (self._mode == self.MODE_VELOCITY):
                # self._left.set_joint_velocities(self._l_joint_command)
                self._right.set_joint_velocities(self._r_joint_command)
            elif (self._mode == self.MODE_TORQUE):
                #self._supp_cuff_int_pubs['left'].publish()
                #self._supp_cuff_int_pubs['right'].publish()
                # self._left.set_joint_torques(self._l_joint_command)
                self._right.set_joint_torques(self._r_joint_command)
            while (time.time() - t1 < self.RMH_delay):
                # idle
                #.01
                time.sleep(0.001)

    def display_worker(self):
        while self._running:
            self.head_display.display_image(self.image,display_rate=100)
            
            t1 = time.time()
            
            while (time.time() - t1 < .01):#self.RMH_delay):
               
                time.sleep(0.001)

    def dio_worker(self):
        while self._running:
            #self.head_display.display_image(self.image,display_rate=100)
            #get dio command
            self.DIOpub.publish(self.h)
            #print(self.h)
            t1 = time.time()
            
            while (time.time() - t1 < .01):#self.RMH_delay):
               
                time.sleep(0.001)

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(
                            description='Initialize Joint Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the Node name
    RR.RobotRaconteurNode.s.NodeName="SawyerRMHServer"

    #Initialize object
    sawyer_obj = Sawyer_impl()

    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
                         RR.IPNodeDiscoveryFlags_LINK_LOCAL | 
                         RR.IPNodeDiscoveryFlags_SITE_LOCAL)

    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()

    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(sawyer_servicedef)
    RR.RobotRaconteurNode.s.RegisterService("Sawyer",
                      "SawyerJoint_Interface.Sawyer",
                                          sawyer_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/SawyerRMHServer/Sawyer"
    raw_input("press enter to quit...\r\n")
    
    sawyer_obj.close()
    
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
     main(sys.argv[1:])
