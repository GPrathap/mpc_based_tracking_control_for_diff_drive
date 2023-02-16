#!/usr/bin/env python
import rospy
import actionlib
from nav_husky_robot.msg import ExecuteControlFeedback,  ExecuteControlResult, ExecuteControlAction
from mpc_control import MPCControl  
import numpy as np 

class MissionActionServer(object):
    def __init__(self, name, controller):
        self._feedback = ExecuteControlFeedback(0.0)
        self._result = ExecuteControlResult(0.0)
        self._action_name = name
        self.controller = controller 
        self._as = actionlib.SimpleActionServer(self._action_name, ExecuteControlAction
                , execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        
    def execute_cb(self, goal):
        r = rospy.Rate(int(1/self.controller.Ts))
        self._feedback.inter_error = 0.0
        _, _, yaw = self.controller.euler_from_quaternion(goal.target_pose.pose.orientation)
        target_pose =  np.array([goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, yaw])
        rospy.loginfo('%s: Executing, moving to a target location %i,%i with current error %i' % 
                      (self._action_name, target_pose[0], target_pose[1], self._feedback.inter_error))
        self.controller.mpc_planner_init(target_pose)
        while not rospy.is_shutdown():
            self.controller.move_one_step()
            if(self.controller.error is not None):
                self._feedback.inter_error = self.controller.error 
            self._as.publish_feedback(self._feedback)
            if self.controller.success:
                self._result.final_error = self.controller.error
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)
                self.controller.success = False 
                self.controller.init_reg = False 
                break 
            r.sleep()
           
          
if __name__ == '__main__':
    rospy.init_node('husky_planner')
    controller = MPCControl(delta_t=0.05, min_error=0.2)
    server = MissionActionServer(rospy.get_name(), controller)
    rospy.spin()