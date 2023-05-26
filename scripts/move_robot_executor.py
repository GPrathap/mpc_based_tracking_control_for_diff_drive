#!/usr/bin/env python
import rospy
import actionlib
from nav_husky_robot.msg import ExecuteControlFeedback,  ExecuteControlResult, ExecuteControlAction
from mpc_control import MPCControl  
import numpy as np 
from datetime import datetime
import csv 
import codecs 

class MissionActionServer(object):
    def __init__(self, name, controller):
        self._feedback = ExecuteControlFeedback(0.0)
        self._result = ExecuteControlResult(0.0)
        self._action_name = name
        self.controller = controller 
        self._as = actionlib.SimpleActionServer(self._action_name, ExecuteControlAction
                , execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.data_set = []
        self.dataset_info = ["x_t", "y_t", "yaw_t", "v_t", "w_t", "xd_t", "yd_t", "yawd_t", "vd_t", "wd_t", "xo_t", "yo_t", "yawo_t", "vo_t", "wo_t", "dt", "timpestamp"]
        with codecs.open("/root/catkin_ws/src/nav_husky_robot/data/dataset_o1.csv", "a", encoding='utf-8') as logfile:
            self.logger = csv.DictWriter(logfile, fieldnames=self.dataset_info)
            self.logger.writeheader()
        logfile.close()
        
    def execute_cb(self, goal):
        r = rospy.Rate(int(1/self.controller.Ts))
        self._feedback.inter_error = 0.0
        _, _, yaw = self.controller.euler_from_quaternion(goal.target_pose.pose.orientation)
        target_pose =  np.array([goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, yaw])
        rospy.loginfo('%s: Executing, moving to a target location %i,%i with current error %i' % 
                      (self._action_name, target_pose[0], target_pose[1], self._feedback.inter_error))
        self.controller.mpc_planner_init(target_pose)
        t_end = rospy.Time.now()
        x_out = 0.0
        u_out = 0.0
        x_pred = 0.0
        u_pred = 0.0
        x_current = 0.0
        u_current = 0.0
        
        t_begin = rospy.Time.now()
        counter = 0
        with codecs.open("/root/catkin_ws/src/nav_husky_robot/data/dataset_o1.csv", "ab", encoding='utf-8') as logfile:
            self.logger = csv.DictWriter(logfile, fieldnames=self.dataset_info)
            while not rospy.is_shutdown():
                t_start = rospy.Time.now()
                duration = t_end - t_start
                timestamp = t_start - t_begin
                if(abs(duration.to_sec()) > self.controller.Ts):
                    x_pred, u_pred = self.controller.move_one_step()
                    if isinstance(x_current, np.ndarray):
                        data_row = np.concatenate((x_current, u_current, x_pred, u_pred, x_out, u_out, np.array([duration.to_sec(), timestamp.to_sec()])), axis=0).tolist()
                        data_info = {}                                     
                        for idx, val in zip(self.dataset_info, data_row):
                            data_info[idx] = val
                        
                        self.logger.writerow(data_info) 

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
                    t_end = rospy.Time.now()
                    x_current, u_current =  self.controller.get_current_pose()
                x_out, u_out = self.controller.get_current_pose()   
        logfile.close()    
           
           
if __name__ == '__main__':
    rospy.init_node('husky_planner')
    controller = MPCControl(delta_t=0.05, min_error=0.4)
    server = MissionActionServer(rospy.get_name(), controller)
    rospy.spin()