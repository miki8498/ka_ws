#ifndef MOVEIT_TUTORIALS_INTERACTIVITY_SRC_POSE_STRING_
#define MOVEIT_TUTORIALS_INTERACTIVITY_SRC_POSE_STRING_

#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>
#include <string>

std::string PoseString(const geometry_msgs::Pose& pose);
std::string PoseString(const Eigen::Isometry3d& pose);

#endif  // MOVEIT_TUTORIALS_INTERACTIVITY_SRC_POSE_STRING_
