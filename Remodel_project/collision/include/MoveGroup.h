#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>

#ifndef MY_PLANNING_CLASS // include guard
#define MY_PLANNING_CLASS

struct Quaternion_s{
    double w, x, y, z;
    void operator=(const Quaternion_s& quat);
};

Quaternion_s ToQuaternion(double yaw, double pitch, double roll);

class MyPlanningClass
    {
        public:
            MyPlanningClass(){};
            ~MyPlanningClass(){};

            moveit_msgs::PlanningScene moveit_planning_scene;
            moveit::planning_interface::PlanningSceneInterface virtual_world;
            const moveit::core::RobotModelPtr kinematic_model;

            bool addMeshObject(std::string obj_id, std::string frame_id, std::string obj_path, geometry_msgs::Pose obj_pose);
            void addBox(std::string blk_name, std::string frame_id, geometry_msgs::Pose pose, std::vector<double> size);            
            void removeObjects(std::string blk_name);
            geometry_msgs::Pose Vector_To_geometryPose(float,float,float,float,float,float);
            
    };

struct Quaternion{
    double w, x, y, z;
};

#endif