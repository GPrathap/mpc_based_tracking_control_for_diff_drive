#!/usr/bin/env python
import rospy
import numpy as np 
import sys 
import tf 
import actionlib
from geometry_msgs.msg import PoseStamped
from nav_husky_robot.msg import ExecuteControlGoal, ExecuteControlAction

class MoveRobot():

    def callback_active(self, ):
        rospy.loginfo("Action server is processing the goal")

    def callback_done(self, state, result):
        rospy.loginfo("Reach to the desired target location")

    def callback_feedback(self, feedback):
        rospy.loginfo("current error:%f" % feedback.inter_error)
        
    def create_goal_message(self, target_location):
        target_pose = PoseStamped()
        target_pose.pose.position.x = target_location[0]
        target_pose.pose.position.y = target_location[1]
        target_pose.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, target_location[2])
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]   
        return ExecuteControlGoal(target_pose)
    
    def execute_mission(self, target_location):
        client = actionlib.SimpleActionClient('husky_planner', ExecuteControlAction)
        client.wait_for_server()
        goal = self.create_goal_message(target_location)
        client.send_goal(goal, active_cb=self.callback_active, feedback_cb=self.callback_feedback,
                        done_cb=self.callback_done)
        client.wait_for_result()
        return client.get_result()

if __name__ == '__main__':
    rospy.init_node('husky_planner_mission')
    try:
        target_pose = np.array([5,5, np.pi/4])
        husky_mission = MoveRobot()
        husky_mission.execute_mission(target_pose)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")