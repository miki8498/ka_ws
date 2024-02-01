#include <ros/ros.h>
#include "../include/MoveGroup.h"
#include <moveit_msgs/CollisionObject.h>
#include "geometric_shapes/shape_operations.h"


void Quaternion_s::operator=(const Quaternion_s& quat){
    x = quat.x;
    y = quat.y;
    z = quat.z;
    w = quat.w;
}

Quaternion_s ToQuaternion(double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion_s q;

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

geometry_msgs::Pose MyPlanningClass::Vector_To_geometryPose(float x,float y,float z, float roll, float pitch,float yaw)
{
    geometry_msgs::Pose obj_pose;

    Quaternion_s KomaxBaseLinkQuaternion;
    KomaxBaseLinkQuaternion = ToQuaternion(roll, pitch, yaw);

    obj_pose.position.x       = x;
    obj_pose.position.y       = y;
    obj_pose.position.z       = z;
    obj_pose.orientation.w    = KomaxBaseLinkQuaternion.w;
    obj_pose.orientation.x    = KomaxBaseLinkQuaternion.x;
    obj_pose.orientation.y    = KomaxBaseLinkQuaternion.y;
    obj_pose.orientation.z    = KomaxBaseLinkQuaternion.z;
    return obj_pose;
}

bool MyPlanningClass::addMeshObject(std::string obj_id, std::string frame_id, std::string obj_path, geometry_msgs::Pose obj_pose)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = obj_id;
    shapes::Mesh* m = shapes::createMeshFromResource(obj_path);
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    collision_object.meshes.resize(1);
    collision_object.mesh_poses.resize(1);
    collision_object.meshes[0] = mesh;
    collision_object.header.frame_id = frame_id;
    collision_object.mesh_poses[0].position.x    = obj_pose.position.x;
    collision_object.mesh_poses[0].position.y    = obj_pose.position.y;
    collision_object.mesh_poses[0].position.z    = obj_pose.position.z;
    collision_object.mesh_poses[0].orientation.w = obj_pose.orientation.w;
    collision_object.mesh_poses[0].orientation.x = obj_pose.orientation.x;
    collision_object.mesh_poses[0].orientation.y = obj_pose.orientation.y;
    collision_object.mesh_poses[0].orientation.z = obj_pose.orientation.z;

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
    collision_object.operation = collision_object.ADD;
   
    bool success;
   
    success = virtual_world.applyCollisionObject(collision_object);
    moveit_planning_scene.world.collision_objects.push_back(collision_object);
 
    if (success){
        ROS_INFO("Obj added into the world");
        return true;
    }

    ROS_INFO("Can0t add obj into the world");
    return false;
}

void MyPlanningClass::addBox(std::string blk_name, std::string frame_id, geometry_msgs::Pose pose, std::vector<double> size)
    {
        moveit_msgs::CollisionObject box;
        //set the relative frame
        box.header.frame_id = frame_id;
        box.id = blk_name;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = size[0];
        primitive.dimensions[1] = size[1];
        primitive.dimensions[2] = size[2];

        geometry_msgs::Pose box_pose;
        box_pose.orientation.x = pose.orientation.x;
        box_pose.orientation.y = pose.orientation.y;
        box_pose.orientation.z = pose.orientation.z;
        box_pose.orientation.w = pose.orientation.w;
        box_pose.position.x = pose.position.x;
        box_pose.position.y = pose.position.y;
        box_pose.position.z = pose.position.z;

        box.primitives.push_back(primitive);
        box.primitive_poses.push_back(box_pose);
        box.operation = box.ADD;

        std::vector<moveit_msgs::CollisionObject> collisionObjects;
        collisionObjects.push_back(box);
        virtual_world.addCollisionObjects(collisionObjects);
        moveit_planning_scene.world.collision_objects.push_back(box);
 
 
        ros::Duration(2).sleep();

        ROS_INFO_STREAM("Added: " << blk_name);
    }



void MyPlanningClass::removeObjects(std::string blk_name)
{
    std::vector<std::string> object_ids;
    object_ids.push_back(blk_name);
    virtual_world.removeCollisionObjects(object_ids);
    ros::Duration(2).sleep();
    ROS_INFO_STREAM("Removed: " << blk_name);
}

