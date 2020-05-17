#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

#from ros import message_operations
#from std_srvs import Empty
#from tf import transform_listener
#from tf import transform_broadcaster
#from tf import transform_datatypes

#FTSensor class definition
#from FTSensor import FTSensor
#include "FTSensor/FTSensor.h"


def datacallback(ftreadings):
    rospy.loginfo("Force : x=[%f] y=[%f] z=[%f] Torque x=[%f] y=[%f] z=[%f]", ftreadings.wrench.force.x, ftreadings.wrench.force.y, ftreadings.wrench.force.z, ftreadings.wrench.torque.x, ftreadings.wrench.torque.y, ftreadings.wrench.torque.z)
    
def listener():
    rospy.init_node('FT_sensor_py')

    rospy.Subscriber("sensor_readings", WrenchStamped , datacallback)
    rospy.spin()

if __name__ == '__main__':
    listener()
