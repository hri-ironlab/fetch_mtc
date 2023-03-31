#include <iostream>
#include "ros/ros.h"
#include "scene_handler/scene_handler.h"
#include <rosparam_shortcuts/rosparam_shortcuts.h>

constexpr char LOGNAME[] = "scene handler";
constexpr char SceneHandler::LOGNAME[];

SceneHandler::SceneHandler()
{
}

SceneHandler::~SceneHandler()
{
}

void SceneHandler::spawnObject(const moveit_msgs::CollisionObject& object)
{
  if (!psi.applyCollisionObject(object))
    throw std::runtime_error("Failed to update object: " + object.id);
}
//const scene_handler::SceneObjects& msg
void SceneHandler::sceneObjectsCallback(const scene_handler::SceneObjects& msg)
{
  std::vector<moveit_msgs::CollisionObject> objects; 

  for(int i = 0; i < msg.objects.size(); i++)
  {
    moveit_msgs::CollisionObject object;
    geometry_msgs::Pose primitive_pose; 
    primitive_pose.position.x = 0; 
    primitive_pose.position.y = 0; 
    primitive_pose.position.z = 0; 
    primitive_pose.orientation.x = 0; 
    primitive_pose.orientation.y = 0; 
    primitive_pose.orientation.z = 0; 
    primitive_pose.orientation.w = 1; 
    if(msg.objects[i].operation == scene_handler::SceneObject::ADD){
      object.id = msg.objects[i].id;
      object.pose = msg.objects[i].spawn_pose; 
      object.header.frame_id = "base_link";
      object.primitives.resize(1);
      

      object.primitive_poses.push_back(primitive_pose);
      object.operation = moveit_msgs::CollisionObject::ADD;

      if(msg.objects[i].type == scene_handler::SceneObject::CYLINDER){
        object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
        object.primitives[0].dimensions = {msg.objects[i].dimensions.z, msg.objects[i].dimensions.x/2}; 
        ROS_INFO("Adding cylinder %d", i);
      }
      else if(msg.objects[i].type == scene_handler::SceneObject::BOX){
        object.primitives[0].dimensions = {msg.objects[i].dimensions.x, msg.objects[i].dimensions.y, msg.objects[i].dimensions.z};
		    object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        ROS_INFO("Adding box %d", i);
      } 
      
      
    }
    else if(msg.objects[i].operation == scene_handler::SceneObject::MOVE){
        object.id = msg.objects[i].id;
        object.header.frame_id = "base_link";
        object.pose = msg.objects[i].spawn_pose;
        object.operation = moveit_msgs::CollisionObject::MOVE;
        ROS_INFO("Moving Object %d", i);
    }
    

    //spawnObject(object);
    objects.push_back(object);
  }
  ROS_INFO("Adding Scene Objects");
  if (!psi.applyCollisionObjects(objects))
    throw std::runtime_error("Failed to update objects");
}


moveit_msgs::CollisionObject SceneHandler::createTable(const std::string& table_name,
                                                       const geometry_msgs::Pose& table_pose,
                                                       const std::string& table_reference_frame,
                                                       std::vector<double>& table_dimensions)
{
  moveit_msgs::CollisionObject table;
  table.id = table_name;
  table.header.frame_id = table_reference_frame;
  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  table.primitives[0].dimensions = table_dimensions;
  table.primitive_poses.push_back(table_pose);
  //table.primitive_poses[0].position.z -= 0.5 * table_dimensions[2];  // align surface with world
  return table;
}

moveit_msgs::CollisionObject SceneHandler::createBox(const std::string& box_name, const geometry_msgs::Pose& box_pose,
                                                     const std::string& box_reference_frame,
                                                     std::vector<double>& box_dimensions)
{
  moveit_msgs::CollisionObject box;
  box.id = box_name;
  box.header.frame_id = box_reference_frame;
  box.primitives.resize(1);
  box.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  box.primitives[0].dimensions = box_dimensions;
  box.primitive_poses.push_back(box_pose);
  //box.primitive_poses[0].position.z += 0.5 * box_dimensions[2];
  return box;
}

moveit_msgs::CollisionObject SceneHandler::createCylinder(const std::string& cylinder_name,
                                                          const geometry_msgs::Pose& cylinder_pose,
                                                          const std::string& cylinder_reference_frame,
                                                          std::vector<double>& cylinder_dimensions)
{
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = cylinder_name;
  cylinder.header.frame_id = cylinder_reference_frame;
  cylinder.primitives.resize(1);
  cylinder.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  cylinder.primitives[0].dimensions = cylinder_dimensions;
  cylinder.primitive_poses.push_back(cylinder_pose);
  cylinder.primitive_poses[0].position.z += 0.5 * cylinder_dimensions[0];
  return cylinder;
}

void SceneHandler::setupStartScene(ros::NodeHandle& pnh)
{
  // Object parameters
  std::string object_name, object_reference_frame;
  std::vector<double> object_dimensions;
  geometry_msgs::Pose object_pose;

  std::size_t error = 0;

  // Spawn a Cylinder
  // error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_name", object_name);
  // error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
  // error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_dimensions", object_dimensions);
  // error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_pose", object_pose);
  // rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  //spawnObject(createCylinder(object_name, object_pose, object_reference_frame, object_dimensions));

  // Spawn a Box 1
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box1_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box1_dimensions", object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box1_pose", object_pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  spawnObject(createBox(object_name, object_pose, object_reference_frame, object_dimensions));

  // Spawn a Box 2
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box2_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box2_dimensions", object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box2_pose", object_pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  spawnObject(createBox(object_name, object_pose, object_reference_frame, object_dimensions));

  // Spawn a Box 3
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box3_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box3_dimensions", object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box3_pose", object_pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  spawnObject(createBox(object_name, object_pose, object_reference_frame, object_dimensions));

  // Spawn a Box 4
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box4_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box4_dimensions", object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "box4_pose", object_pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  spawnObject(createBox(object_name, object_pose, object_reference_frame, object_dimensions));

  // Spawn a table
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "table_dimensions", object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "table_pose", object_pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  spawnObject(createTable(object_name, object_pose, object_reference_frame, object_dimensions));
}
