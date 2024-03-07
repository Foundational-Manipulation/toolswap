#!/usr/bin/env python3
import rospy
import actionlib
from toolswap.msg import SwapToolAction, SwapToolGoal
from quest2ros.msg import OVR2ROSInputs
from std_msgs.msg import String

class Orchestrator:
    def __init__(self):
        # publisher/listener for toolswap action server
        self.toolswap_client = actionlib.SimpleActionClient("swap_tool",SwapToolAction)
        self.toolswap_client.wait_for_server()

        # publisher for policy
        self.policy_publisher = rospy.Publisher("/orchestrator", OVR2ROSInputs, queue_size=5)
        # listener for VLM Action
        vlm_sub = rospy.Subscriber("/vlm/action", String, self.vlm_callback, queue_size=1)

        self.msg_template = OVR2ROSInputs()
        self.msg_template.button_upper = False # Sausages
        self.msg_template.button_lower = False # Rice
        self.msg_template.press_index = 0.0 #1.0: Lid
        self.msg_template.press_middle = 0.0 #1.0: Beer
        self.msg_template.thumb_stick_horizontal = 0.0
        self.msg_template.thumb_stick_vertical = 0.0 #1.0: Stop policy


    def vlm_callback(self, msg):
        if msg.data == "SCOOP_RICE":
            # Publish stop to policy
            policy_msg = self.msg_template
            policy_msg.thumb_stick_vertical = 1.0
            self.policy_publisher.publish(policy_msg)

            # Change tool
            goal = SwapToolGoal("spoon")
            self.toolswap_client.send_goal(goal)
            self.toolswap_client.wait_for_result()
            #TODO: check result
            print(self.toolswap_client.get_result())

            # Publish new policy (starts automatically)
            policy_msg.thumb_stick_vertical = 0.0
            policy_msg.button_lower = True
            self.policy_publisher.publish(policy_msg)

        if msg.data == "PICK_SAUSAGE":
            # Publish stop to policy
            policy_msg = self.msg_template
            policy_msg.thumb_stick_vertical = 1.0
            self.policy_publisher.publish(policy_msg)

            # Change tool
            goal = SwapToolGoal("gripper")
            self.toolswap_client.send_goal(goal)
            self.toolswap_client.wait_for_result()
            #TODO: check result
            print(self.toolswap_client.get_result())

            # Publish new policy (starts automatically)
            policy_msg.thumb_stick_vertical = 0.0
            policy_msg.button_upper = True
            self.policy_publisher.publish(policy_msg)

        if msg.data == "OPEN_BEER":
            # Publish stop to policy
            policy_msg = self.msg_template
            policy_msg.thumb_stick_vertical = 1.0
            self.policy_publisher.publish(policy_msg)

            # Change tool
            goal = SwapToolGoal("beer_opener")
            self.toolswap_client.send_goal(goal)
            self.toolswap_client.wait_for_result()
            #TODO: check result
            print(self.toolswap_client.get_result())

            # Publish new policy (starts automatically)
            policy_msg.thumb_stick_vertical = 0.0
            policy_msg.press_middle = 1.0
            self.policy_publisher.publish(policy_msg)

        if msg.data == "REMOVE_LID":
            # Publish stop to policy
            policy_msg = self.msg_template
            policy_msg.thumb_stick_vertical = 1.0
            self.policy_publisher.publish(policy_msg)

            # Change tool
            goal = SwapToolGoal("none")
            self.toolswap_client.send_goal(goal)
            self.toolswap_client.wait_for_result()
            #TODO: check result
            print(self.toolswap_client.get_result())

            # Publish new policy (starts automatically)
            policy_msg.thumb_stick_vertical = 0.0
            policy_msg.press_index = 1.0
            self.policy_publisher.publish(policy_msg)

if __name__ == "__main__":
    rospy.init_node("orchestrator")
    rate = rospy.Rate(10)
    orchestrator = Orchestrator()
    rospy.spin()
    