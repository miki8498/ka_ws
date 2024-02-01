#include <ros/ros.h>
#include "Spline.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FIRServer");
    uc1_c_spline::Spline server(500);
    return 0;
}