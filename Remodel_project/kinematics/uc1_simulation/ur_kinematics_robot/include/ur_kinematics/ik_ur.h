/* 
 * Author: Giorgio Medico
 *
 * Created on April 9, 2023
 * 
 * UR Forward Kinematics and Inverse Kinematics
 * 
 * Based on the paper: A General Analytical Algorithm for Collaborative Robot with 6 Degrees of Freedom
 * 
 * Authors: S. Chen, M. Luo, O. Abdelaziz and G. Jiang
 *  
 */

#ifndef IK_ROBOTICS_H
#define IK_ROBOTICS_H

#include <ros/ros.h>
#include <stdio.h>  
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <math.h> 
#include <cmath>
#include <Eigen/Dense>
#include "ur_kinematics_robot/UrInverseKinematics.h"
#include "ur_kinematics_robot/UrForwardKinematics.h"
#include <sensor_msgs/JointState.h>

#include <boost/thread.hpp>


namespace inverse_kinem
{

    class FIKServer
    {

        public:

            FIKServer(int rate); // Class Constructor
            virtual ~FIKServer(); // Class Destructor
            void run();

        private:

            //Variables
            int rate;
            int num_sols;
            double d1, d4, d5,d6;
            double a2, a3;
            double alph1, alph4, alph5;

            //vector of joint limits                1      1       2    2      3     3          4     4         5     5       6      6
            std::vector<double> joint_limits = {-2*M_PI, 2*M_PI, -M_PI, M_PI, -M_PI, M_PI, -2*M_PI, 2*M_PI, -2*M_PI, 2*M_PI, -2*M_PI, 2*M_PI};
           
            // Eigen::Matrix4d T_0_6 = Eigen::Matrix4d::Identity();
            // Eigen::Matrix<double,6,1> q;
            // Eigen::Matrix<double,8,6> ik_solutions = Eigen::Matrix<double,8,6>::Zero();

            ros::NodeHandle n;
            ros::ServiceServer fk_service ;
            ros::ServiceServer ik_service ;
            std::string ur_type;
            ur_kinematics_robot::UrInverseKinematics::Response res_new;
            std::vector<double> max_error;
            bool verbose;
            std::vector<double> last_joints;
            Eigen::Matrix3d Rx_fk, Rz_fk;

            //Methods
            void initialiseRos();
            void coefficients();

            bool fk_cb(ur_kinematics_robot::UrForwardKinematics::Request &req, ur_kinematics_robot::UrForwardKinematics::Response &res);
            bool ik_cb(ur_kinematics_robot::UrInverseKinematics::Request &req, ur_kinematics_robot::UrInverseKinematics::Response &res);

            Eigen::Matrix4d HTrans(std::vector<double> q);
            Eigen::Matrix4d AH(double alpha, double a, double d, double theta);
            Eigen::Matrix<double,8,6> invKine(Eigen::Matrix4d desired_pos);

            bool checkq6();

    };
}

#endif //IK_ROBOTICS_H