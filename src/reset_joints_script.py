import rospy
import actionlib
import roslib
import numpy as np
from numpy.linalg import norm
import scipy
from scipy.spatial.transform import Rotation as R
from toolswap.msg import SwapToolAction
from franka_gripper.msg import FGMoveGoal
from geometry_msgs.msg import PoseStamped, Twist
from controller_manager_msgs.srv import *

class ToolswapServer:
    # TODO: Measure tool setpoints
    tool_dict = {
        "beer_opener": {
            "setpoints": list((
                np.array([0.024384731937221016, 0.9977838259807481, -0.06175409590285157, 0.0, 
                        0.9995208627465637, -0.023187616604000816, 0.02002813700269917, 0.0, 
                        0.01855217805321463, -0.062214085780904815, -0.9978903868761979, 0.0, 
                        0.5956395224336262, -0.30191009132292196, 0.25379345470194965, 1.0]).reshape(4,4).transpose(),
                np.array([0.024384731937221016, 0.9977838259807481, -0.06175409590285157, 0.0, 
                        0.9995208627465637, -0.023187616604000816, 0.02002813700269917, 0.0, 
                        0.01855217805321463, -0.062214085780904815, -0.9978903868761979, 0.0, 
                        0.5956395224336262, -0.30191009132292196, 0.20379345470194965, 1.0]).reshape(4,4).transpose(),
                "gripper",
                np.array([0.024384731937221016, 0.9977838259807481, -0.06175409590285157, 0.0, 
                        0.9995208627465637, -0.023187616604000816, 0.02002813700269917, 0.0, 
                        0.01855217805321463, -0.062214085780904815, -0.9978903868761979, 0.0, 
                        0.5956395224336262, -0.30191009132292196, 0.20379345470194965, 1.0]).reshape(4,4).transpose(),
                np.array([0.024384731937221016, 0.9977838259807481, -0.06175409590285157, 0.0, 
                        0.9995208627465637, -0.023187616604000816, 0.02002813700269917, 0.0, 
                        0.01855217805321463, -0.062214085780904815, -0.9978903868761979, 0.0, 
                        0.5956395224336262, -0.20191009132292196, 0.20379345470194965, 1.0]).reshape(4,4).transpose()
            )),
            "is_in_rack": True
        },
        "spoon": {
            "setpoints": list((np.array([0.03766387361374579, 0.9981809816518387, -0.04687117871558548, 0.0, 
                                        0.9991981124836109, -0.037015803660572666, 0.014618795769824118, 0.0, 
                                        0.01285747710969215, -0.04738510608888363, -0.9987939412127604, 0.0, 
                                        0.47088288551213475, -0.30125472084044413, 0.25771810556363646, 1.0]).reshape(4,4).transpose(),
                              np.array([0.03766387361374579, 0.9981809816518387, -0.04687117871558548, 0.0, 
                                        0.9991981124836109, -0.037015803660572666, 0.014618795769824118, 0.0, 
                                        0.01285747710969215, -0.04738510608888363, -0.9987939412127604, 0.0, 
                                        0.47088288551213475, -0.30125472084044413, 0.20471810556363646, 1.0]).reshape(4,4).transpose(),
                              "gripper",
                              np.array([0.03766387361374579, 0.9981809816518387, -0.04687117871558548, 0.0, 
                                        0.9991981124836109, -0.037015803660572666, 0.014618795769824118, 0.0, 
                                        0.01285747710969215, -0.04738510608888363, -0.9987939412127604, 0.0, 
                                        0.47088288551213475, -0.30125472084044413, 0.20471810556363646, 1.0]).reshape(4,4).transpose(),
                              np.array([0.03766387361374579, 0.9981809816518387, -0.04687117871558548, 0.0, 
                                        0.9991981124836109, -0.037015803660572666, 0.014618795769824118, 0.0, 
                                        0.01285747710969215, -0.04738510608888363, -0.9987939412127604, 0.0, 
                                        0.47088288551213475, -0.15125472084044413, 0.20471810556363646, 1.0]).reshape(4,4).transpose()
                            )),
            "is_in_rack": True
        },
        "gripper": {
            "setpoints": list((np.array([-0.01716291815444826, 0.9998351792396254, -0.003974354853070617, 0.0, 
                                        0.9992725376132986, 0.017287253406245318, 0.03370895048433735, 0.0, 
                                        0.03377275045905942, -0.0033929850266445905, -0.999423778473896, 0.0, 
                                        0.34752298199666265, -0.2881940018156153, 0.2570503174381176, 1.0]).reshape(4,4).transpose(),
                              np.array([-0.01716291815444826, 0.9998351792396254, -0.003974354853070617, 0.0, 
                                        0.9992725376132986, 0.017287253406245318, 0.03370895048433735, 0.0, 
                                        0.03377275045905942, -0.0033929850266445905, -0.999423778473896, 0.0, 
                                        0.34752298199666265, -0.2881940018156153, 0.2070503174381176, 1.0]).reshape(4,4).transpose(),
                              "gripper",
                              np.array([-0.01716291815444826, 0.9998351792396254, -0.003974354853070617, 0.0, 
                                        0.9992725376132986, 0.017287253406245318, 0.03370895048433735, 0.0, 
                                        0.03377275045905942, -0.0033929850266445905, -0.999423778473896, 0.0, 
                                        0.34752298199666265, -0.2881940018156153, 0.2070503174381176, 1.0]).reshape(4,4).transpose(),
                              np.array([-0.01716291815444826, 0.9998351792396254, -0.003974354853070617, 0.0, 
                                        0.9992725376132986, 0.017287253406245318, 0.03370895048433735, 0.0, 
                                        0.03377275045905942, -0.0033929850266445905, -0.999423778473896, 0.0, 
                                        0.34752298199666265, -0.1881940018156153, 0.2070503174381176, 1.0]).reshape(4,4).transpose()
                            )),
            "is_in_rack": True
        },
    }
    current_tool = None

    start_setpoint = np.array([0.9964996077818302, 0.022431589003332808, 0.08041207873604908, 0.0, 
                               0.017456925702225012, -0.9979112237219678, 0.062041858579186204, 0.0, 
                               0.08163738513881041, -0.06042210336888351, -0.9948288831614095, 0.0, 
                               0.5415816895385864, -0.014602715530858734, 0.43957066082080576, 1.0]).reshape(4,4).transpose()
    start_joints_rice = np.array([-0.25, -0.41514149873344698, 0.0, -2.251524366217463, 0.0, 1.8579533827751187, 0.5474793668364485])
    start_joints_standard = np.array([0.0, -0.41514149873344698, 0.0, -2.251524366217463, 0.0, 1.8579533827751187, 0.8474793668364485])
    start_joints = np.array([0.0, -0.41514149873344698, 0.0, -2.251524366217463, 0.0, 1.8579533827751187, 0.8474793668364485])



    def __init__(self):
        self._server = actionlib.SimpleActionServer("swap_tool",SwapToolAction,self.swap_tool,False)
        self._server.start()
        
        # Load cartesian pose controller
        rospy.wait_for_service("/controller_manager/load_controller")    
        load_controller_srv = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
        resp = load_controller_srv.call(LoadControllerRequest("kth_cartesian_pose_effort_interface_controller"))
        if resp.ok:
            print("Loaded", "kth_cartesian_pose_effort_interface_controller")
        else:
            print("Error when loading", "kth_cartesian_pose_effort_interface_controller")

        # Controller switching service
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_controller_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        # Publishers to controllers
        self._cartesian_pose_pub = rospy.Publisher("/kth_cartesian_pose_effort_interface_controller/panda/equilibrium_pose",PoseStamped, queue_size=1 )
        self._gripper_ros_pub = rospy.Publisher("/fg_franka_gripper/fg_franka_gripper_goal", FGMoveGoal, queue_size=1)
        self._joint_pose_pub = rospy.Publisher("/kth_joint_pose_effort_interface_controller/panda/equilibrium_pose",PoseStamped, queue_size=1 )

    def _swap_controller(self, name_start, name_stop):
        resp = self.switch_controller_srv.call([name_start], [name_stop], 2, False ,0.0)
        if resp.ok == 1:
            print ("Started " + name_start + " successfully")
            print ("Stopped " + name_stop + " successfully")
        else:
            print ("Error when  " + name_start + " starting")
            print ("Error when  " + name_stop + " stopping")

    def _goto_setpoint(self, setpoint_matrix):
        position = setpoint_matrix[:3, 3]
        quat = R.from_matrix(setpoint_matrix[:3, :3]).as_quat()
        print(f"Incoming setpoint matrix: {setpoint_matrix}")
        print(f"Extracted position: {position}")
        print(f"Extracted quaternion: {quat}")

        panda_EE=PoseStamped()
        panda_EE.pose.position.x=position[0]
        panda_EE.pose.position.y=position[1]
        panda_EE.pose.position.z=position[2] 
        panda_EE.pose.orientation.x=quat[0]
        panda_EE.pose.orientation.y=quat[1]
        panda_EE.pose.orientation.z=quat[2]
        panda_EE.pose.orientation.w=quat[3]
        self._cartesian_pose_pub.publish(panda_EE)
        
        rospy.sleep(0.1) #Race condition? First message might still have error for previous setpoint
        # panda_x_error_ik does not seem to return error with actual measurements! Instead with planner? This means if the
        # controller dies/robot stops moving (or didn't even start moving), the error keeps being updated and goes down.
        error = rospy.wait_for_message('/kth_cartesian_pose_effort_interface_controller/panda_x_error_ik', PoseStamped)
        pos_error = np.array([error.pose.position.x, error.pose.position.y, error.pose.position.z])
        print(norm(pos_error))
        while norm(pos_error) > 1e-10:
            rospy.sleep(0.1)
            error = rospy.wait_for_message('/kth_cartesian_pose_effort_interface_controller/panda_x_error_ik', PoseStamped)
            pos_error = np.array([error.pose.position.x, error.pose.position.y, error.pose.position.z])
            print(norm(pos_error))

    def _visit_tool(self, tool, mode):

        if mode is "load":
            for setpoint in self.tool_dict[tool]["setpoints"]:
                if setpoint is "gripper":
                    self._gripper(mode)
                else:
                    self._goto_setpoint(setpoint)
            self.tool_dict[tool]["is_in_rack"] = False
            return True
        elif mode is "unload":
            for setpoint in reversed(self.tool_dict[tool]["setpoints"]):
                if setpoint is "gripper":
                    self._gripper(mode)
                else:
                    self._goto_setpoint(setpoint)
            self.tool_dict[tool]["is_in_rack"] = True
            return True
        else:
            print("Only modes 'load' and 'unload' are available for _visit_tool(). Your mode is ", mode)
            return False

    def _gripper(self, mode):
        gripper_goal = FGMoveGoal()
        if mode is "load":
            gripper_goal.width = 0.01
        if mode is "unload":
            gripper_goal.width = 0.07
        gripper_goal.velocity = 0.1
        self._gripper_ros_pub.publish(gripper_goal)
        rospy.sleep(3)

    def swap_tool(self, msg):

        self._swap_controller('kth_cartesian_pose_effort_interface_controller', 'kth_joint_pose_effort_interface_controller') 



        # Return to safe position
        self._goto_setpoint(self.start_setpoint)

        self._swap_controller('kth_joint_pose_effort_interface_controller', 'kth_cartesian_pose_effort_interface_controller') 

        # Reset joints
        init_joint_pose = PoseStamped()
        init_joint_pose.pose.position.x = self.start_joints[0]
        init_joint_pose.pose.position.y = self.start_joints[1]
        init_joint_pose.pose.position.z = self.start_joints[2]
        init_joint_pose.pose.orientation.x = self.start_joints[3]
        init_joint_pose.pose.orientation.y = self.start_joints[4]
        init_joint_pose.pose.orientation.z = self.start_joints[5]
        init_joint_pose.pose.orientation.w = self.start_joints[6]
        self._joint_pose_pub.publish(init_joint_pose)
        
        # TODO: Error checking to see if we have arrived at joint configuration. For now just a hack to wait
        rospy.sleep(5)

        self._server.set_succeeded()

if __name__ == "__main__":
    rospy.init_node("toolswap_server")
    rate = rospy.Rate(10)
    server = ToolswapServer()
    rospy.spin()
    