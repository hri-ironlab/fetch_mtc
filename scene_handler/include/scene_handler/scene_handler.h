#ifndef SCENEHANDLER_H
#define SCENEHANDLER_H

#include <iostream>
#include "ros/ros.h"

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningSceneWorld.h>


#include "scene_handler/SceneObject.h"
#include <scene_handler/SceneObjects.h>

class SceneHandler {
	public:
		SceneHandler();
		virtual ~SceneHandler();
		
		moveit_msgs::CollisionObject createTable(const std::string& table_name, const geometry_msgs::Pose& table_pose,const std::string& table_reference_frame, std::vector<double>& table_dimensions);
		moveit_msgs::CollisionObject createBox(const std::string& box_name, const geometry_msgs::Pose& box_pose,const std::string& box_reference_frame, std::vector<double>& box_dimensions);
		moveit_msgs::CollisionObject createCylinder(const std::string& cylinder_name, const geometry_msgs::Pose& cylinder_pose, const std::string& cylinder_reference_frame, std::vector<double>& cylinder_dimensions);
		void spawnObject(const moveit_msgs::CollisionObject& object);
		void setupStartScene(ros::NodeHandle& pnh);
		void sceneObjectsCallback(const scene_handler::SceneObjects& msg); 

	private:
		static constexpr char LOGNAME[]{ "task planner" };
		// object + surface
		std::vector<std::string> support_surfaces_;
		std::string object_reference_frame_;
		std::string surface_link_;
		std::string world_frame_;
		std::vector<double> box_dimensions_;
		std::vector<double> cylinder_dimensions_;
		moveit::planning_interface::PlanningSceneInterface psi;
		ros::NodeHandle pnh_;
};
#endif
