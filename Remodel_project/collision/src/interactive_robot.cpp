#include <interactive_robot.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/robot_state/conversions.h>

// default world object position is just in front and left of Panda robot.
const Eigen::Isometry3d
    InteractiveRobot::DEFAULT_WORLD_OBJECT_POSE_(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.9, 0.1)));

// size of the world geometry cube
const double InteractiveRobot::WORLD_BOX_SIZE_ = 0.15;

// minimum delay between calls to callback function
const ros::Duration InteractiveRobot::min_delay_(0.01);

InteractiveRobot::InteractiveRobot(const std::string& robot_description, const std::string& robot_topic,
                                   const std::string& marker_topic, const std::string& imarker_topic)
  : user_data_(nullptr)
  , nh_()  // this node handle is used to create the publishers
  // create publishers for markers and robot state
  , robot_state_publisher_(nh_.advertise<moveit_msgs::DisplayRobotState>(robot_topic, 1))
  , world_state_publisher_(nh_.advertise<visualization_msgs::Marker>(marker_topic, 100))
  // create an interactive marker server for displaying interactive markers
  , interactive_marker_server_(imarker_topic)
  , imarker_robot_(nullptr)
  , imarker_world_(nullptr)
  // load the robot description
  , rm_loader_(robot_description)
  , group_(nullptr)
{
  // get the RobotModel loaded from urdf and srdf files
  robot_model_ = rm_loader_.getModel();
  if (!robot_model_)
  {
    ROS_ERROR("Could not load robot description");
    throw RobotLoadException();
  }

  // create a RobotState to keep track of the current robot pose
  robot_state_.reset(new moveit::core::RobotState(robot_model_));
  if (!robot_state_)
  {
    ROS_ERROR("Could not get RobotState from Model");
    throw RobotLoadException();
  }
  robot_state_->setToDefaultValues();

  //Prepare to move the "panda_arm" group
  group_ = robot_state_->getJointModelGroup("ur5_arm");
  std::string end_link = group_->getLinkModelNames().back();
  std::cout<< end_link << std::endl;
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link);  //sembra giusto il fatto di prendere l'ulitmo link della catena )ur5_finger_tip
  std::cout<<desired_group_end_link_pose_.translation()<<std::endl;
  std::cout<<desired_group_end_link_pose_.rotation()<<std::endl;

  // Create a marker to control the "panda_arm" group
  imarker_robot_ = new IMarker(interactive_marker_server_, "robot", desired_group_end_link_pose_, "ur5_base_link",
                               boost::bind(movedRobotMarkerCallback, this, boost::placeholders::_1), IMarker::BOTH);

  // create an interactive marker to control the world geometry (the yellow cube)
  desired_world_object_pose_ = DEFAULT_WORLD_OBJECT_POSE_;
  imarker_world_ = new IMarker(interactive_marker_server_, "world", desired_world_object_pose_, "world",
                               boost::bind(movedWorldMarkerCallback, this, boost::placeholders::_1), IMarker::POS);

  // start publishing timer.
  init_time_ = ros::Time::now();
  last_callback_time_ = init_time_;
  average_callback_duration_ = min_delay_;
  schedule_request_count_ = 0;
  publish_timer_ = nh_.createTimer(average_callback_duration_, &InteractiveRobot::updateCallback, this, true);

  // begin publishing robot state
  scheduleUpdate();
}

InteractiveRobot::~InteractiveRobot()
{
  delete imarker_world_;
  // delete imarker_robot_;
}

// callback called when marker moves.  Moves right hand to new marker pose.
void InteractiveRobot::movedRobotMarkerCallback(InteractiveRobot* robot,
                                                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  Eigen::Isometry3d pose;
  tf2::fromMsg(feedback->pose, pose);
  robot->setGroupPose(pose);
}

// callback called when marker moves.  Moves world object to new pose.
void InteractiveRobot::movedWorldMarkerCallback(InteractiveRobot* robot,
                                                const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  Eigen::Isometry3d pose;
  tf2::fromMsg(feedback->pose, pose);
  robot->setWorldObjectPose(pose);
}

// set the callback timer to fire if needed.
// Return true if callback should happen immediately
bool InteractiveRobot::setCallbackTimer(bool new_update_request)
{
  publish_timer_.stop();

  const ros::Time now = ros::Time::now();
  const ros::Duration desired_delay = std::max(min_delay_, average_callback_duration_ * 1.2);
  ros::Duration sec_since_last_callback = now - last_callback_time_;
  ros::Duration sec_til_next_callback = desired_delay - sec_since_last_callback;

  if (schedule_request_count_)
  {
    // need a callback desired_delay seconds after previous callback
    schedule_request_count_ += new_update_request ? 1 : 0;
    if (sec_til_next_callback <= ros::Duration(0.0001))
    {
      // just run the callback now
      return true;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (new_update_request)
  {
    if (sec_til_next_callback < min_delay_)
    {
      // been a while.  Use min_delay_.
      // Set last_callback_time_ to prevent firing too early
      sec_til_next_callback = min_delay_;
      sec_since_last_callback = desired_delay - sec_til_next_callback;
      last_callback_time_ = now - sec_since_last_callback;
    }
    publish_timer_.setPeriod(sec_til_next_callback);
    publish_timer_.start();
    return false;
  }
  else if (!init_time_.isZero())
  {
    // for the first few seconds after startup call the callback periodically
    // to ensure rviz gets the initial state.
    // Without this rviz does not show some state until markers are moved.
    if ((now - init_time_).sec >= 8)
    {
      init_time_ = ros::Time(0, 0);
      return false;
    }
    else
    {
      publish_timer_.setPeriod(std::max(ros::Duration(1.0), average_callback_duration_ * 2));
      publish_timer_.start();
      return false;
    }
  }
  else
  {
    // nothing to do.  No callback needed.
    return false;
  }
}

// Indicate that the world or the robot has changed and
// the new state needs to be updated and published to rviz
void InteractiveRobot::scheduleUpdate()
{
  // schedule an update callback for the future.
  // If the callback should run now, call it.
  if (setCallbackTimer(true))
    updateCallback(ros::TimerEvent());
}

/* callback called when it is time to publish */
void InteractiveRobot::updateCallback(const ros::TimerEvent& /*event*/)
{
  ros::Time tbegin = ros::Time::now();
  publish_timer_.stop();

  // do the actual calculations and publishing
  updateAll();

  // measure time spent in callback for rate limiting
  ros::Time tend = ros::Time::now();
  average_callback_duration_ = (average_callback_duration_ + (tend - tbegin)) * 0.5;
  last_callback_time_ = tend;
  schedule_request_count_ = 0;

  // schedule another callback if needed
  setCallbackTimer(false);
}

/* Calculate new positions and publish results to rviz */
void InteractiveRobot::updateAll()
{
  publishWorldState();

  if (robot_state_->setFromIK(group_, desired_group_end_link_pose_, 0.1))
  {
    publishRobotState();

    if (user_callback_)
      user_callback_(*this);
  }
}

// change which group is being manipulated
void InteractiveRobot::setGroup(const std::string& name)
{
  const moveit::core::JointModelGroup* group = robot_state_->getJointModelGroup(name);
  if (!group)
  {
    ROS_ERROR_STREAM("No joint group named " << name);
    if (!group_)
      throw RobotLoadException();
  }
  group_ = group;
  std::string end_link = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link);
  if (imarker_robot_)
  {
    imarker_robot_->move(desired_group_end_link_pose_);
  }
}

// return current group name
const std::string& InteractiveRobot::getGroupName() const
{
  return group_->getName();
}

/* remember new desired robot pose and schedule an update */
void InteractiveRobot::setGroupPose(const Eigen::Isometry3d& pose)
{
  desired_group_end_link_pose_ = pose;
  scheduleUpdate();
}

/* publish robot pose to rviz */
void InteractiveRobot::publishRobotState()
{
  moveit_msgs::DisplayRobotState msg;
  moveit::core::robotStateToRobotStateMsg(*robot_state_, msg.state);
  robot_state_publisher_.publish(msg);
}

/* remember new world object position and schedule an update */
void InteractiveRobot::setWorldObjectPose(const Eigen::Isometry3d& pose)
{
  desired_world_object_pose_ = pose;
  scheduleUpdate();
}

/* publish world object position to rviz */
void InteractiveRobot::publishWorldState()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "ur5e_base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "world_box";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = WORLD_BOX_SIZE_;
  marker.scale.y = WORLD_BOX_SIZE_;
  marker.scale.z = WORLD_BOX_SIZE_;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.4f;
  marker.lifetime = ros::Duration();
  marker.pose = tf2::toMsg(desired_world_object_pose_);
  world_state_publisher_.publish(marker);
}

/* get world object pose and size */
void InteractiveRobot::getWorldGeometry(Eigen::Isometry3d& pose, double& size)
{
  pose = desired_world_object_pose_;
  size = WORLD_BOX_SIZE_;
}