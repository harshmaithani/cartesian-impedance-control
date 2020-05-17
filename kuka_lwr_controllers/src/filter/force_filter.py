#!/usr/bin/env python

import collections
import numpy as np
import rospy
import geometry_msgs.msg



class ForceFilter(object):
    """
    Publishes the Baxter's inverse Jacobian.

    """
    def __init__(self):
        # Params
        self.force_in = None
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 1000.0))

        # Number of samples for the filter
        self.samples = rospy.get_param('~samples', 200)
        self.deque = collections.deque([], maxlen=self.samples)

        # Publishers
        self.force_out = rospy.Publisher(
            '/sensor_readings_filtered', geometry_msgs.msg.WrenchStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber('/sensor_readings', geometry_msgs.msg.WrenchStamped, self.wrench_in_cb)

    def wrench_in_cb(self, msg):
        """
        Obtains the first pose.
        """
        self.force_in = msg
	

    def start(self):
        force_out = geometry_msgs.msg.WrenchStamped()
        while not rospy.is_shutdown():
            if self.force_in:      
                self.deque.append(self.force_in.wrench.force.x)
                if len(self.deque) == self.samples:
                    force = np.ma.average(self.deque)             
                    force_out.header = self.force_in.header
                    force_out.wrench = self.force_in.wrench
                    force_out.wrench.force.x = force 
                    self.force_out.publish(force_out)

                self.force_in = None
            self.loop_rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('force_filter')
        force_filter = ForceFilter()
        force_filter.start()
    except rospy.ROSInterruptException:
        pass

