#! /usr/bin/env python
#! /usr/bin/env python
import rospy
import argparse
import numpy
import intera_interface
import math
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)


from RobotRaconteur.Client import *


class ImpedanceControllerClass():


	def __init__(self):
		self.poirot=RRN.ConnectService('tcp://localhost:39309/SawyerRMHServer/Sawyer')
		self.captain=RRN.ConnectService('tcp://localhost:39309/SawyerRMHServer/Sawyer')

		rospy.Subscriber("spacenav/twist",Twist, self.SpaceNavCallback)
		rospy.Subscriber("spacenav/joy",Joy, self.SpaceNavJoyCallback)

		# functions beginning with get_ are functions that should be called in the main program. gets a value of a function, regardless of what the function is
		#other functions can be edited without fear of messing up the overall structure. Similar to changing the definition of a function. as long as
		#the changed function returns the same number and types of variables, the get_ function should still work


		#initialize variables
		self.CalibratePoint = [0,0,0,0,0,0,0]
		self.StartingPioint=[0,0,.1,.1,.1,.1,.1]
		self.neutralposition=[0,-1.18,0,2.175,0,.566,0]
		self.calibratedelay=5
		self.neutralflag=1

		self.returntoneutral=0
		self.Fxe_raw=0
		self.Fye_raw=0
		self.Fze_raw=0
		self.prevjointvelocities=numpy.array([0,0,0,0,0,0])
		self.rate = 1/20  # Hz


		#offsets for force sensor calibration
		self.offsetFxe=0
		self.offsetFye=0
		self.offsetFze=0

		self.PrevInvJac=numpy.zeros((6,6))

		self.EEx_lvec=[]
		self.EEy_lvec=[]
		self.EEz_lvec=[]
		self.EEx_avec=[]
		self.EEy_avec=[]
		self.EEz_avec=[]

		self.avgxl=0
		self.avgyl=0
		self.avgzl=0
		self.avgxa=0
		self.avgya=0
		self.avgza=0

		self.Jxl_raw=0
		self.Jyl_raw=0
		self.Jzl_raw=0

		self.Jxa_raw=0
		self.Jya_raw=0
		self.Jza_raw=0

		self.leftbutton=0 #use left button to move last joint on robot arm
		self.rightbutton=0 #hold right button to reset to neutral position
        print("init")
