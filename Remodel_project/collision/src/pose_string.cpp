#include <pose_string.h>

// tf2
#include <tf2_eigen/tf2_eigen.h>

#include <iostream>
#include <iomanip>

/** return a string describing a pose (position and quaternion) */
std::string PoseString(const geometry_msgs::Pose& pose)
{
  std::stringstream ss;
  ss << "p(";
  ss << std::setprecision(3);
  ss << std::setiosflags(std::ios::fixed);
  ss << std::setw(7) << pose.position.x << ", ";
  ss << std::setw(7) << pose.position.y << ", ";
  ss << std::setw(7) << pose.position.z << ") q(";
  ss << std::setw(7) << pose.orientation.x << ", ";
  ss << std::setw(7) << pose.orientation.y << ", ";
  ss << std::setw(7) << pose.orientation.z << ", ";
  ss << std::setw(7) << pose.orientation.w << ")";

  return std::string(ss.str());
}

/** return a string describing a pose
 * Assumes the pose is a rotation and a translation
 */
std::string PoseString(const Eigen::Isometry3d& pose)
{
  return PoseString(tf2::toMsg(pose));
}