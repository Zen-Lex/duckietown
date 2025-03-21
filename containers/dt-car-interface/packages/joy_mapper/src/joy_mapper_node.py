#!/usr/bin/env python3

import rospy
import math

from duckietown_msgs.msg import Twist2DStamped, BoolStamped, FSMState
from duckietown_msgs.srv import ChangePattern
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from duckietown.dtros import DTROS, NodeType, TopicType


# Button List index of joy.buttons array:
#   0: A
#   1: B
#   2: X
#   3: Y (Emergency Stop)
#   4: Left Back
#   5: Right Back
#   6: Toggle Joystick Control Mode
#   7: Toggle Start LF
#   8: Logitek
#   9: Left joystick
#   10: Right joystick


class JoyMapperNode(DTROS):
    """Interprets the Joystick commands.

    The `JoyMapperNode` receives :obj:`Joy` messages from a phisical joystick or a virtual one,
    interprets the buttons presses and acts accordingly.

    **Joystick bindings:**

    +----------------------+------------------+------------------------------------------------+
    | Physical joystick    | Virtual joystick | Action                                         |
    +======================+==================+================================================+
    | Directional controls | Arrow keys       | Move the Duckiebot (if not in lane-following)  |
    +----------------------+------------------+------------------------------------------------+
    | Start button         | `A` key          | Start lane-following                           |
    +----------------------+------------------+------------------------------------------------+
    | Back button          | `S` key          | Stop lane-following                            |
    +----------------------+------------------+------------------------------------------------+
    | Y button             | `E` key          | Toggle Emergency Stop                          |
    +----------------------+------------------+------------------------------------------------+

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~speed_gain (:obj:`float`): Gain for the directional joystick keys (forward/reverse)
        ~steer_gain (:obj:`int`): Gain for the directional joystick keys (steering angle)
        ~bicycle_kinematics (:obj:`bool`): `True` for bicycle kinematics; `False` for holonomic
           kinematics. Default is `False`
        ~simulated_vehicle_length (:obj:`float`): Used in bicycle kinematics model

    Subscriber:
        joy (:obj:`Joy`): The command read from joystick
        emergency_stop (:obj:`BoolStamped`): The emergency stop status

    Publishers:
        ~car_cmd (:obj:`duckietown_msgs/Twist2DStamped`): Wheels command for Duckiebot, based
           on the directional buttons pressed
        ~joystick_override (:obj:`duckietown_msgs/BoolStamped`): Boolean that is used to control
           whether lane-following or joystick control is on
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(JoyMapperNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # emergency stop disabled by default
        self.e_stop = False

	# Joystick Control disabled by default
        self.joy_override = False
        self.idle = True
        self.fsm_mode = "IDLE_MODE"

        #self._mode_topic = rospy.get_param("~mode_topic")
        self.speed_modifier = {
          "IDLE_MODE": [0, 0],
          "RECOVERY_MODE": [0.2, 0.5],
          "LANE_FOLLOWING": [0, 0],
          "NORMAL_JOYSTICK_CONTROL": [1, 1]
        }

        # Add the node parameters to the parameters dictionary
        self._speed_gain = rospy.get_param("~speed_gain")
        self._steer_gain = rospy.get_param("~steer_gain")
        self._bicycle_kinematics = rospy.get_param("~bicycle_kinematics")
        self._simulated_vehicle_length = rospy.get_param("~simulated_vehicle_length")

        # Publications
        self.pub_car_cmd = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        self.pub_joy_override = rospy.Publisher(
            "~joystick_override", BoolStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        self.pub_idle = rospy.Publisher(
            "~idle_mode", BoolStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        self.pub_e_stop = rospy.Publisher(
            "~emergency_stop", BoolStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        self.changePattern = rospy.ServiceProxy("led_emitter_node/set_pattern", ChangePattern)

        # Subscription to the joystick command
        self.sub_joy = rospy.Subscriber("~joy", Joy, self.joy_cb, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.estop_cb, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.change_mode_cb, queue_size=1)

    def estop_cb(self, estop_msg):
        """
        Callback that process the received :obj:`BoolStamped` messages.

        Args:
            estop_msg (:obj:`BoolStamped`): the emergency_stop message to process.
        """
        self.e_stop = estop_msg.data

    def joy_cb(self, joy_msg):
        """
        Callback that process the received :obj:`Joy` messages.

        Args:
            joy_msg (:obj:`Joy`): the joystick message to process.
        """
        # Navigation buttons
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.get_rostime()
        # Left stick V-axis. Up is positive
        car_cmd_msg.v = joy_msg.axes[1] * self._speed_gain * self.speed_modifier[self.fsm_mode][0]
        if self._bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = joy_msg.axes[3] * self._steer_gain
            car_cmd_msg.omega = car_cmd_msg.v / self._simulated_vehicle_length * math.tan(steering_angle) * self.speed_modifier[self.fsm_mode][1]
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = joy_msg.axes[3] * self._steer_gain * self.speed_modifier[self.fsm_mode][1]
        self.pub_car_cmd.publish(car_cmd_msg)

        # Back button: Toggle Joystick Control (key 'S')
        if joy_msg.buttons[6] == 1:
            if self.fsm_mode not in ["IDLE_MODE", "NORMAL_JOYSTICK_CONTROL"]:
                return

            self.joy_override = not self.joy_override
            override_msg = BoolStamped()
            override_msg.header.stamp = joy_msg.header.stamp
            override_msg.data = self.joy_override
            self.log(f"Joystick Control: {'ON' if self.joy_override else 'OFF'}")
            self.pub_joy_override.publish(override_msg)

        # Start button: Toggle LF (key 'A')
        elif joy_msg.buttons[7] == 1:
            if self.e_stop or self.fsm_mode == "NORMAL_JOYSTICK_CONTROL":
                return

            if self.fsm_mode == "RECOVERY_MODE":
                self.idle = True
                self.log("Recovery Mode: OFF")
            else:
                self.idle = not self.idle
                self.log(f"Autonomus Driving (LF): {'OFF' if self.idle else 'ON'}")

            idle_msg = BoolStamped()
            idle_msg.header.stamp = joy_msg.header.stamp
            idle_msg.data = self.idle
            self.pub_idle.publish(idle_msg)

        # Y button: Toggle Emergency Stop (key 'E')
        elif joy_msg.buttons[3] == 1:
            self.e_stop = not self.e_stop
            estop_msg = BoolStamped()
            estop_msg.header.stamp = joy_msg.header.stamp
            estop_msg.data = self.e_stop
            self.log(f"Emergency Stop: {'ON' if self.e_stop else 'OFF'}")
            self.pub_e_stop.publish(estop_msg)
        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                self.logwarn("No binding for joy_msg.buttons = %s" % str(joy_msg.buttons))

    def change_mode_cb(self, fsm_state_msg):
        self.fsm_mode = fsm_state_msg.state
        self.log(f"Speed modifier for Joystick Control updated to: {self.speed_modifier[self.fsm_mode]}")

if __name__ == "__main__":
    # Initialize the node with rospy
    joy_mapper = JoyMapperNode(node_name="joy_mapper")
    # Keep it spinning to keep the node alive
    rospy.spin()
