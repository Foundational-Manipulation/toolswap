#!/usr/bin/env python3
import rospy
import actionlib
import roslib
import numpy as np
from toolswap.msg import SwapToolAction
from geometry_msgs.msg import PoseStamped
from controller_manager_msgs.srv import *

class JointResetServer:
    start_joints_rice = np.array([-0.25, -0.41514149873344698, 0.0, -2.251524366217463, 0.0, 1.8579533827751187, 0.5474793668364485])
    start_joints_standard = np.array([0.0, -0.41514149873344698, 0.0, -2.251524366217463, 0.0, 1.8579533827751187, 0.8474793668364485])

    def __init__(self):
        self._server = actionlib.SimpleActionServer("reset_joints", SwapToolAction, self.reset_joints, False)
        self._server.start()
        
        # Controller switching service
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_controller_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        # Publishers to controllers
        self._joint_pose_pub = rospy.Publisher("/kth_joint_pose_effort_interface_controller/panda/equilibrium_pose",PoseStamped, queue_size=1 )

    def _swap_controller(self, name_start, name_stop):
        resp = self.switch_controller_srv.call([name_start], [name_stop], 2, False ,0.0)
        if resp.ok == 1:
            print ("Started " + name_start + " successfully")
            print ("Stopped " + name_stop + " successfully")
        else:
            print ("Error when  " + name_start + " starting")
            print ("Error when  " + name_stop + " stopping")

    def reset_joints(self, msg):
        self._swap_controller('kth_joint_pose_effort_interface_controller', 'kth_cartesian_velocity_effort_interface_controller') 

        if msg.tool_name == "rice":
            requested_joint = self.start_joints_rice
        else:
            requested_joint = self.start_joints_standard

        # Reset joints
        init_joint_pose = PoseStamped()
        init_joint_pose.pose.position.x = requested_joint[0]
        init_joint_pose.pose.position.y = requested_joint[1]
        init_joint_pose.pose.position.z = requested_joint[2]
        init_joint_pose.pose.orientation.x = requested_joint[3]
        init_joint_pose.pose.orientation.y = requested_joint[4]
        init_joint_pose.pose.orientation.z = requested_joint[5]
        init_joint_pose.pose.orientation.w = requested_joint[6]
        self._joint_pose_pub.publish(init_joint_pose)
        
        # TODO: Error checking to see if we have arrived at joint configuration. For now just a hack to wait
        rospy.sleep(5)

        self._swap_controller('kth_cartesian_velocity_effort_interface_controller', 'kth_joint_pose_effort_interface_controller') 
        self._server.set_succeeded()

if __name__ == "__main__":
    rospy.init_node("joint_reset_server")
    rate = rospy.Rate(10)
    server = JointResetServer()
    rospy.spin()
    