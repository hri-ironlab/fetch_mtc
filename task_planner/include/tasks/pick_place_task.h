#ifndef PICKPLACETASK_H
#define PICKPLACETASK_H

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <task_planner/PlanPickPlaceAction.h>
#include <task_planner/PickPlace.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>

#include <task_planner/task_parameters.h>
#include <tasks/task_base.h>

using namespace moveit::task_constructor;

class PickPlaceTask : public TaskBase
{
public:
  PickPlaceTask(const std::string& task_name);
  ~PickPlaceTask() = default;
  bool init(const TaskParameters& parameters);
  bool plan();
  bool preempt();
  bool execute();
  void getSolutionMsg(moveit_task_constructor_msgs::Solution& solution);

private:
  TaskPtr task_;
  const std::string task_name_;
  Stage* current_state_stage_;
  Stage* attach_object_stage_;
};

#endif