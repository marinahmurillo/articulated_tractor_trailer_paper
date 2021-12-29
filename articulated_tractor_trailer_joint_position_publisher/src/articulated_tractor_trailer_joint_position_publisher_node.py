#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from articulated_tractor_trailer_joint_position_publisher.msg import JointPosition

class JointPositionPublisher:

    def __init__(self):

        rospy.loginfo("Initializating a Joint State Publisher node.")

        # ROS Rate defaults at 5Hz unless is set by parameter.
        publish_frequency = rospy.get_param('~publish_frequency', 5.0)
        control_period = 1.0 / publish_frequency

        if rospy.has_param('~joint_name'):
            self._joint_name = rospy.get_param('~joint_name')
        else:
            raise rospy.exceptions.ROSInitException("Joint name not set.")

        rospy.loginfo(("This node will publish %s position.") % self._joint_name)

        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        self._joint_sub = rospy.Subscriber('joint_states', JointState, self.joint_states_callback)

        # TODO: Hay que elegir el mensaje que más nos guste
        self._joint_pub = rospy.Publisher(self._joint_name + "_position", Float64, queue_size=1)
        self._joint_pub_custom = rospy.Publisher(self._joint_name + "_position_custom", JointPosition, queue_size=1)

        rospy.Timer(rospy.Duration(control_period), self.timerCallback)


    def timerCallback(self, *args):

        joints_found = []
        positions = []
        velocities = []
        efforts = []

        (found, position, velocity, effort) = self.return_joint_state(self._joint_name)
        joints_found.append(found)
        positions.append(position)
        velocities.append(velocity)
        efforts.append(effort)

        self._joint_pub.publish(position)

        joint_position_msg = JointPosition()
        joint_position_msg.header.stamp = rospy.Time.now()
        joint_position_msg.header.frame_id = self._joint_name
        joint_position_msg.found = found
        joint_position_msg.position = position

        self._joint_pub_custom.publish(joint_position_msg)


    # Callback function for the joint state subscriber:
    # when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort

    # Returns (found, position, velocity, effort) for the joint joint_name
    # (found is 1 if found, 0 otherwise)
    def return_joint_state(self, joint_name):

        # no messages yet
        if self.name == []:
            rospy.logerr("No robot_state messages received!\n")
            return (0, 0., 0., 0.)

        # return info for this joint
        if joint_name in self.name:
            index = self.name.index(joint_name)
            position = self.position[index]
            velocity = self.velocity[index]
            effort = self.effort[index]

        # unless it's not found
        else:
            rospy.logerr("Joint %s not found!", (joint_name,))
            return (0, 0., 0., 0.)
        return (1, position, velocity, effort)


if __name__ == '__main__':
    rospy.init_node("articulated_tractor_trailer_joint_position_publisher")
    JointPositionPublisher()
    rospy.spin()
