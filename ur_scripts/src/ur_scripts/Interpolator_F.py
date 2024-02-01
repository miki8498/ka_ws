#!/usr/bin/env python3.8

import rospy
from uc1_c_spline.srv          import *
from geometry_msgs.msg         import Pose

class Interpolator_:
    def __init__(self) -> None:
        self.Interpolator_service = rospy.ServiceProxy('interpolate_linear', interpolate)
        self.Interpolator_service.wait_for_service()
        
    def interpolate(self, RATE, startpose, goalpose, period):
        
        if(startpose == goalpose):
            print("Start and goal pose are the same")
            pose = Pose()
            pose.position.x = startpose.position.x
            pose.position.y = startpose.position.y
            pose.position.z = startpose.position.z
            pose.orientation.x = startpose.orientation.x
            pose.orientation.y = startpose.orientation.y
            pose.orientation.z = startpose.orientation.z
            pose.orientation.w = startpose.orientation.w
            
            return [pose]
        
        goal = interpolateRequest()
        
        goal.start_pose.position.x = startpose.position.x
        goal.start_pose.position.y = startpose.position.y
        goal.start_pose.position.z = startpose.position.z

        goal.start_pose.orientation.x = startpose.orientation.x
        goal.start_pose.orientation.y = startpose.orientation.y
        goal.start_pose.orientation.z = startpose.orientation.z
        goal.start_pose.orientation.w = startpose.orientation.w

        goal.goal_pose.position.x = goalpose.position.x
        goal.goal_pose.position.y = goalpose.position.y
        goal.goal_pose.position.z = goalpose.position.z

        goal.goal_pose.orientation.x = goalpose.orientation.x
        goal.goal_pose.orientation.y = goalpose.orientation.y
        goal.goal_pose.orientation.z = goalpose.orientation.z
        goal.goal_pose.orientation.w = goalpose.orientation.w
        
        goal.rate     = RATE       #rate cambia in base al robot (UR5 = 125, UR5e = 500 )
        goal.vel_cm_s = period
       
        res = self.Interpolator_service.call(goal)

        if(res.success == True):
            trajectory = res.trajectory
            return trajectory
        else:   
            rospy.logerr("Error in interpolation")
            

 