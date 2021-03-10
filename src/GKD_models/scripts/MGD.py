#!/usr/bin/env python

from __future__ import print_function
from math import *
import yaml
from sensor_msgs.msg import JointState
from MGD.srv import MGD,MGDResponse
import rospy
import numpy as np

#Paramètres du robot :
# mi = masse i
# li = longueur du bras i
# Izi = Inertie selon l'axe z
# ci = longueur entre la liaison et le centre de masse du bras i

wwith open('~/ros/src/integ_gcd_models_pkg/RobotParam.yml') as f :
	yaml_dict = yaml.safe_load(f)
	l1 = yaml_dict.get("l1")
	l2 = yaml_dict.get("l2")




def handle_MGD(req):
	theta = req.input.position
	pos[0] = l1*cos(theta[0])+l2*cos(theta[0]+theta[1])
	pos[1] = l1*sin(theta[0])+l2*sin(theta[0]+theta[1])
	
    return MGDResponse(pos)
    

def MGD_server():
    rospy.init_node('MGD_server')
    s = rospy.Service('MGD', MGD, handle_MGD)
    print("MGD")
    rospy.spin()

if __name__ == "__main__":
    MGD_server()
