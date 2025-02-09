#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped


class PilotNode(DTROS):

    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.CONTROL)

        vehicle_name = os.environ['VEHICLE_NAME']
        topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(topic, WheelsCmdStamped, queue_size=1)

        self.trim = rospy.get_param(f"/{vehicle_name}/kinematics_node/trim")

    def goStraight(self):
        vel = 0.5
        calibratedVelLeft = vel * (1 - self.trim)
        calibratedVelRight = vel * (1 + self.trim)
        message = WheelsCmdStamped(vel_left=calibratedVelLeft, vel_right=calibratedVelRight)
        self._publisher.publish(message)
        rospy.sleep(1)

    def turnLeft(self):
        message = WheelsCmdStamped(vel_left=1.0, vel_right=1.0)
        self._publisher.publish(message)
        rospy.sleep(2)

    def turnRight(self):
        message = WheelsCmdStamped(vel_left=1.0, vel_right=1.0)
        self._publisher.publish(message)
        rospy.sleep(2)

    def stay(self):
        stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self._publisher.publish(stop)
        rospy.sleep(2)

    def run(self):
        while not rospy.is_shutdown():
            self.goStraight()
            self.stay()

            rospy.sleep(10)

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self._publisher.publish(stop)


if __name__ == '__main__':
    node = PilotNode(node_name='pilot_node')
    node.run()
    rospy.spin()