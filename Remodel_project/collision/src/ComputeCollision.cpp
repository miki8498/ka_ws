#include <ros/ros.h>
#include "../include/ComputeCollision.h"

//cronometro
#include <chrono>

using namespace collision_checker;

    CollisionServer::CollisionServer(int rate, std::string robot_1, std::string robot_2, const std::string planning_group, spawn_coordinates spawn_table, std::vector<double> size_table,spawn_coordinates spawn_wall, std::vector<double> size_wall)
    : visual_tools()
    , move_group(planning_group)
    , robot_model_loader("robot_description")
    , kinematic_model(robot_model_loader.getModel())
    , planning_scene(std::make_shared<planning_scene::PlanningScene>(kinematic_model))
    , state(planning_scene->getCurrentStateNonConst())
    , acm(planning_scene->getAllowedCollisionMatrix())
    , psm(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description"))
    {   
        
        this->rate = rate;
        this->robot_1 = robot_1;
        this->robot_2 = robot_2;
        this->addCollisionObjects(spawn_table, size_table, spawn_wall, size_wall);
        this->initializeRos();
        this->run();
    }

    CollisionServer::~CollisionServer()
    {
    }

    void CollisionServer::run()
    {
        ros::Rate rs(this->rate);
        while(ros::ok())
        {
            ros::spinOnce();
            rs.sleep();
        }
    }

    void CollisionServer::initializeRos()
    {   
    
        this->collision_checker_server = this->n.advertiseService("collision_check", &CollisionServer::collision_check_cb,this);
        visual_tools.setRobotStateTopic("interactive_robot_state");
        planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create(),true);

        planning_scene->setPlanningSceneMsg(this->moveit_planning_scene);

        for (this->it2 = res1.contacts.begin(); this->it2 != res1.contacts.end(); ++this->it2)
        {
          acm.setEntry(this->it2->first.first, this->it2->first.second, true);
        }
        state.setToDefaultValues();
    }

    void CollisionServer::addCollisionObjects(spawn_coordinates spawn_table, std::vector<double> size_table, spawn_coordinates spawn_wall, std::vector<double> size_wall)
    {
        planning_scene->getPlanningSceneMsg(this->moveit_planning_scene);
        this->addBox("wall", "world", this->Vector_To_geometryPose(spawn_table.x, spawn_table.y, spawn_table.z, spawn_table.rx, spawn_table.ry, spawn_table.rz), size_table);
        this->addBox("table", "world", this->Vector_To_geometryPose(spawn_wall.x, spawn_wall.y, spawn_wall.z, spawn_wall.rx, spawn_wall.ry, spawn_wall.rz), size_wall);
        acm.setEntry("ur_left_base_link_inertia", "table", true);
        acm.setEntry("ur_right_base_link_inertia", "table", true);
        // acm.print(std::cout);
    }

   
    bool CollisionServer::collision_check_cb(collision::CollisionServer::Request &req, collision::CollisionServer::Response &res)
    {   
        this->n_robots = req.n_robots;
        this->check = false;
        int index = -1;
        
        if(this->n_robots == 1){
            this->ur_type = req.ur_type;
            
            if(this->ur_type == fmt::format("{}", robot_1)){ 
                this->second_robot_arm = fmt::format("{}_arm", robot_2);
                this->second_robot_hand = fmt::format("{}_hand", robot_2);}
            else{
                this->second_robot_arm = fmt::format("{}_arm", robot_1);
                this->second_robot_hand = fmt::format("{}_hand", robot_1);}    

            this->fix_joint(this->second_robot_arm, this->second_robot_hand);

            for (unsigned int i = 0; i < req.joint_config.size(); i++)
            {   
                
                state.setJointPositions(fmt::format("{}_shoulder_pan_joint", ur_type), &req.joint_config[i].data[0]);
                state.setJointPositions(fmt::format("{}_shoulder_lift_joint",ur_type), &req.joint_config[i].data[1]);
                state.setJointPositions(fmt::format("{}_elbow_joint",        ur_type), &req.joint_config[i].data[2]);
                state.setJointPositions(fmt::format("{}_wrist_1_joint",      ur_type), &req.joint_config[i].data[3]);
                state.setJointPositions(fmt::format("{}_wrist_2_joint",      ur_type), &req.joint_config[i].data[4]);
                state.setJointPositions(fmt::format("{}_wrist_3_joint",      ur_type), &req.joint_config[i].data[5]);
                state.setJointPositions(fmt::format("{}_bl_to_leftFinger",   ur_type), &req.gripper_config[0]);   
                state.setJointPositions(fmt::format("{}_leftFinger_to_rightFinger",   ur_type), &req.gripper_config[1]);      

                state.update();
                visual_tools.publishRobotState(state);
                
                bool env_collision  = this-> collision_with_environment(state);
                bool self_collision = this-> selfCollision(state);
           
                res1.print();
                res2.print();
                if(env_collision || self_collision)
                {
                    index = i;
                    this->check = true;
                    break;
                }
            }
            if(this->check)
            {
                res.collision_index = index;
                res.success = false;
                this->check = false; 
            }
            else
            {
                res.collision_index = index;
                res.success = true;
                this->check = false;
            }
            
        }
        else{      

            this->check = false;
            for (unsigned int i = 0, j = 0; i < req.joint_config_right.size(), j < req.joint_config_left.size(); i++, j++)
            {   
            
                state.setJointPositions(fmt::format("{}_shoulder_pan_joint" , robot_1),  &req.joint_config_right[i].data[0]);
                state.setJointPositions(fmt::format("{}_shoulder_lift_joint", robot_1),  &req.joint_config_right[i].data[1]);
                state.setJointPositions(fmt::format("{}_elbow_joint"        , robot_1),  &req.joint_config_right[i].data[2]);
                state.setJointPositions(fmt::format("{}_wrist_1_joint"      , robot_1),  &req.joint_config_right[i].data[3]);
                state.setJointPositions(fmt::format("{}_wrist_2_joint"      , robot_1),  &req.joint_config_right[i].data[4]);
                state.setJointPositions(fmt::format("{}_wrist_3_joint"      , robot_1),  &req.joint_config_right[i].data[5]);
                state.setJointPositions(fmt::format("{}_bl_to_leftFinger"   , robot_1),  &req.gripper_config[0]);   
                state.setJointPositions(fmt::format("{}_leftFinger_to_rightFinger", robot_1), &req.gripper_config[1]);   
                state.setJointPositions(fmt::format("{}_shoulder_pan_joint" , robot_2), &req.joint_config_left[i].data[0]);
                state.setJointPositions(fmt::format("{}_shoulder_lift_joint", robot_2), &req.joint_config_left[i].data[1]);
                state.setJointPositions(fmt::format("{}_elbow_joint"        , robot_2), &req.joint_config_left[i].data[2]);
                state.setJointPositions(fmt::format("{}_wrist_1_joint"      , robot_2), &req.joint_config_left[i].data[3]);
                state.setJointPositions(fmt::format("{}_wrist_2_joint"      , robot_2), &req.joint_config_left[i].data[4]);
                state.setJointPositions(fmt::format("{}_wrist_3_joint"      , robot_2), &req.joint_config_left[i].data[5]);
                state.setJointPositions(fmt::format("{}_bl_to_leftFinger"   , robot_2), &req.gripper_config[0]);   
                state.setJointPositions(fmt::format("{}_leftFinger_to_rightFinger", robot_2), &req.gripper_config[1]);      
                state.update();
                visual_tools.publishRobotState(state);
                
                bool env_collision  = this-> collision_with_environment(state);
                bool self_collision = this-> selfCollision(state);

                res1.print();
                res2.print();

                if(env_collision || self_collision)
                {
                    index = i;
                    this->check = true;
                    break;
                }

            }

            if(this->check)
            {
                res.collision_index = index;
                res.success = false;
                this->check = false;
            }
            else
            {
                res.collision_index = index;
                res.success = true;
                this->check = false;
            }
        }

        return true;         
    }
    
    std::string CollisionServer::removeLastN(std::string &str, int n) 
    {
        return str.erase(str.length() - n);
    }

    void CollisionServer::fix_joint(std::string ur_arm, std::string ur_hand)
    {

        this->move_group.getCurrentState()->copyJointGroupPositions(state.getJointModelGroup(fmt::format("{}",ur_arm)),this->point_arm.positions);
        this->move_group.getCurrentState()->copyJointGroupPositions(state.getJointModelGroup(fmt::format("{}",ur_hand)),this->point_hand.positions);
        int n_char = 4;  
        std::string type = removeLastN(ur_arm, n_char);
        state.setJointPositions(fmt::format("{}_shoulder_pan_joint", type), &this->point_arm.positions[0]);
        state.setJointPositions(fmt::format("{}_shoulder_lift_joint",type), &this->point_arm.positions[1]);
        state.setJointPositions(fmt::format("{}_elbow_joint",        type), &this->point_arm.positions[2]);
        state.setJointPositions(fmt::format("{}_wrist_1_joint",      type), &this->point_arm.positions[3]);
        state.setJointPositions(fmt::format("{}_wrist_2_joint",      type), &this->point_arm.positions[4]);
        state.setJointPositions(fmt::format("{}_wrist_3_joint",      type), &this->point_arm.positions[5]);   
        state.setJointPositions(fmt::format("{}_bl_to_leftFinger",   type), &this->point_hand.positions[0]);   
        state.setJointPositions(fmt::format("{}_leftFinger_to_rightFinger", type), &this->point_hand.positions[1]);    
    }

    bool CollisionServer::collision_with_environment(robot_state::RobotState& state)
    {
        this->req1.contacts = true;
        this->req1.max_contacts = 1;
        this->req1.max_contacts_per_pair = 1;
        this->res1.clear();

        planning_scene->getCollisionEnv()->checkRobotCollision(this->req1, this->res1, state, acm);
        return this->res1.collision;
    }

    bool CollisionServer::selfCollision(robot_state::RobotState& state)
    {
        this->req2.contacts = true;
        this->req2.max_contacts = 1;
        this->req2.max_contacts_per_pair = 1;
        this->res2.clear();
        planning_scene->getCollisionEnv()->checkSelfCollision(this->req2, this->res2, state, acm);
    
        return this->res2.collision;
    }
