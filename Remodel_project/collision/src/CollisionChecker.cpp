#include "../include/ComputeCollision.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "collision_checker");
   ros::AsyncSpinner spinner(1); 
   spinner.start();

   std::string robot_1 = "ur_right";
   std::string robot_2 = "ur_left";
   const std::string planning_group = "dual";
   spawn_coordinates spawn_table;
   spawn_table.x  = 0;
   spawn_table.y  = 0;
   spawn_table.z  = 0;
   spawn_table.rx = -1.57;
   spawn_table.ry = 0;
   spawn_table.rz = -1.57;
   std::vector<double> size_table = {1.5, 0.75, 0.001};
   spawn_coordinates spawn_wall;
   spawn_table.x  = 0;
   spawn_table.y  = -0.375;
   spawn_table.z  = 0.375;
   spawn_table.rx = -1.57;
   spawn_table.ry = 0;
   spawn_table.rz = 0;
   std::vector<double> size_wall = {1.5, 0.75, 0.001};
   collision_checker::CollisionServer collision_checker(1000, robot_1, robot_2, planning_group, spawn_table, size_table, spawn_wall, size_wall);
  
  
   return 0;
}
