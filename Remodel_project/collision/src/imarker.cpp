#include <imarker.h>
#include <pose_string.h>
#include <tf2_eigen/tf2_eigen.h>

#include <utility>

/* default callback which just prints the current pose */
void IMarker::printFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM(feedback->marker_name.c_str() << "is now at :" << PoseString(feedback->pose));
}

/* create 1 cylinder as part of a 3-cylinder axis (used to visualize pose
 * manipulation) */
visualization_msgs::Marker IMarker::makeAxisCyl(IMarker::Axis axis)
{
  visualization_msgs::Marker marker;
  double scale = imarker_.scale * 0.4;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = scale * 0.15;
  marker.scale.y = scale * 0.15;
  marker.scale.z = scale;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  switch (axis)
  {
    case X:
      marker.pose.position.x = 0.5 * scale;
      marker.pose.orientation.y = 1.0;
      marker.color.r = 0.5;
      break;
    case Y:
      marker.pose.position.y = 0.5 * scale;
      marker.pose.orientation.x = 1.0;
      marker.color.g = 0.5;
      break;
    case Z:
    default:
      marker.pose.position.z = 0.5 * scale;
      marker.color.b = 0.5;
      break;
  }

  return marker;
}

/* create a positional control no marker */
void IMarker::makeBallControl()
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  imarker_.controls.push_back(control);
}

/* create an orientation/position control with an axis marker */
void IMarker::makeAxisControl()
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeAxisCyl(X));
  control.markers.push_back(makeAxisCyl(Y));
  control.markers.push_back(makeAxisCyl(Z));
  imarker_.controls.push_back(control);
}

/* move to new pose */
void IMarker::move(const Eigen::Isometry3d& pose)
{
  imarker_.pose = tf2::toMsg(pose);
  server_->applyChanges();
}

/* initialize the marker.  All constructors call this. */
void IMarker::initialize(interactive_markers::InteractiveMarkerServer& server, const std::string& name,
                         const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                         const std::string& frame_id,
                         boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&)> callback,
                         IMarker::Dof dof)
{
  server_ = &server;
  imarker_.header.frame_id = frame_id;
  imarker_.pose.position = tf2::toMsg(position);
  imarker_.pose.orientation = tf2::toMsg(orientation);
  imarker_.scale = 0.3;

  imarker_.name = name;
  imarker_.description = name;

  // insert a control with a marker
  switch (dof)
  {
    case POS:
      makeBallControl();
      break;
    case ORIENT:
    case BOTH:
    default:
      makeAxisControl();
      break;
  }

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  // add orientation and/or position controls
  for (int i = 0; i < 3; i++)
  {
    static const char* dirname = "xyz";

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 0;
    switch (i)
    {
      case 0:
        control.orientation.x = 1;
        break;
      case 1:
        control.orientation.y = 1;
        break;
      case 2:
        control.orientation.z = 1;
        break;
    }

    if (dof == ORIENT || dof == BOTH)
    {
      control.name = std::string("rotate_") + std::string(1, dirname[i]);
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      imarker_.controls.push_back(control);
    }
    if (dof == POS || dof == BOTH)
    {
      control.name = std::string("move_") + std::string(1, dirname[i]);
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      imarker_.controls.push_back(control);
    }
  }

  // tell the server to show the marker and listen for changes
  server_->insert(imarker_);
  server_->setCallback(imarker_.name, std::move(callback));
  server_->applyChanges();
}