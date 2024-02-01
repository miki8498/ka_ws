#ifndef MAIN_H
#define MAIN_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "interactive_robot.h"

#include <moveit_msgs/PlanningScene.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include "std_msgs/Bool.h"
#include "fmt/format.h"
#include "collision/CollisionServer.h"
#include "collision/CollisionDistance.h"
#include "../include/MoveGroup.h"
#include <collision/InfoData.h>
#include <vector>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>

struct spawn_coordinates
{
   double x, y ,z, rx, ry, rz;
};

namespace collision_checker{
    class CollisionServer : public MyPlanningClass  //se una classe inerited ha un costruttore, lo devo mettere con gli altri dopo i : MyPlanningClass()
    {
        public:

        CollisionServer(int rate, std::string robot_1, std::string robot_2, const std::string planning_group, spawn_coordinates spawn_table, std::vector<double> size_table,spawn_coordinates spawn_wall, std::vector<double> size_wall);
        ~CollisionServer();

        void run();

        std::uint32_t n_robots;
      
        ros::NodeHandle n; 
        ros::ServiceServer collision_checker_server;

        std::string removeLastN(std::string &str, int n);
        void fix_joint(std::string ur_arm, std::string ur_hand);
        bool collision_with_environment(robot_state::RobotState& state);
        bool selfCollision(robot_state::RobotState& state);


        private:

        int rate;
        std::string robot_1;
        std::string robot_2;
        
        bool check;
       
        std::string ur_type;
        std::string second_robot_arm;
        std::string second_robot_hand;

        std::vector<double> joint_values;

        ros::Subscriber joint_state;

        moveit_visual_tools::MoveItVisualTools visual_tools;
        moveit::planning_interface::MoveGroupInterface move_group;
        const robot_model_loader::RobotModelLoader robot_model_loader;
        moveit::core::RobotModelPtr kinematic_model;
        std::shared_ptr<planning_scene::PlanningScene> planning_scene;
        trajectory_msgs::JointTrajectoryPoint point_arm;  
        trajectory_msgs::JointTrajectoryPoint point_hand; 
        robot_state::RobotState& state;
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm;

        
        collision_detection::CollisionResult::ContactMap::const_iterator it2;
        collision_detection::AllowedCollisionMatrix acm;

        collision_detection::CollisionResult res1;
        collision_detection::CollisionRequest req1;
        collision_detection::CollisionResult res2;
        collision_detection::CollisionRequest req2;

        void initializeRos();
        void addCollisionObjects(spawn_coordinates spawn_table, std::vector<double> size_table, spawn_coordinates spawn_wall, std::vector<double> size_wall);
        
        bool collision_check_cb(collision::CollisionServer::Request &req, collision::CollisionServer::Response &res);
    };

}

#endif // MAIN_H
