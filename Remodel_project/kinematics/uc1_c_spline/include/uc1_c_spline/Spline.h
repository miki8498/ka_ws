

#ifndef SPLINE_H
#define SPLINE_H

#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <FIRFilter.h>
#include <string>

#include <uc1_c_spline/interpolate.h>
#include <uc1_c_spline/joint_interpolate.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>


using Vector6d = Eigen::Matrix<double,6,1>;


namespace uc1_c_spline
{

    class Spline 
    {
        public:
        Spline(int rate);			//Class Constructor #1
        virtual ~Spline();			//Class Destructor
        
        
        private:
        ros::NodeHandle n;
        ros::ServiceServer interpolate_service;
        ros::ServiceServer interpolate_service_linear;
        ros::ServiceServer joint_interpolate_service;


        

        std::vector<fir_filter::FIRFilter*> filter;

        float rate_spline;
        int slerp_steps;
        float period;
        
        geometry_msgs::Pose start_pose;
        geometry_msgs::Pose goal_pose;
        std::vector<geometry_msgs::Pose> trajectory;

        std_msgs::Float64MultiArray start_joints;
        std::vector<std_msgs::Float64MultiArray> joints_trajectory;
        


        //methods
        void initialiseRos();
        // void initialiseServer();
        Eigen::Quaterniond quaternion_negation(Eigen::Quaterniond v);

        void initialiseFilters(float period);
        void initialiseFiltersLinear(float period);
        void initialiseFiltersJoints(float period);

        bool interpolate_serviceCB(uc1_c_spline::interpolate::Request &req, uc1_c_spline::interpolate::Response &res);
        bool joint_interpolate_serviceCB(uc1_c_spline::joint_interpolate::Request &req, uc1_c_spline::joint_interpolate::Response &res);
        bool interpolate_service_linearCB(uc1_c_spline::interpolate::Request &req, uc1_c_spline::interpolate::Response &res);
    };


}

#endif /* SPLINE_H */

