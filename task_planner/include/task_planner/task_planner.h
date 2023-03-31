#ifndef TASKPLANNER_H
#define TASKPLANNER_H

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
#include <task_planner/PlanPickPlaceAction.h>
#include <task_planner/PlanPickPlaceGoal.h>
#include <actionlib/server/simple_action_server.h>
#include "task_planner/PickPlace.h"
#include <task_planner/task_parameters.h>
#include "tasks/pick_place_task.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace moveit::task_constructor;

typedef actionlib::SimpleActionServer<task_planner::PlanPickPlaceAction> PickPlaceServer;

class TaskPlanner
{
public:
  TaskPlanner();
  ~TaskPlanner() = default;

  void executePickPlaceCallback(const task_planner::PlanPickPlaceGoalConstPtr& goal);
  void preemptPickPlaceCallback();
  void planRobotActionCallback(const task_planner::PickPlaceConstPtr& msg);
  void setParameters(TaskParameters parameters);
  void loadParameters(const ros::NodeHandle& pnh_);

  void TestPickPlace(); 
  ros::Publisher plan_status;
private:
  void initializeServer();
  std::unique_ptr<PickPlaceServer> pick_place_server_;

  // Which transform to use
  int transform_orientation;
  std::vector<double> diagonal_frame_transform {0.02, 0, 0.0, 0, 3.9275, 0.0};  
  std::vector<double> horizontal_frame_transform {0.02, 0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> vertical_frame_transform {0.02, 0, 0.0,  0, 4.713, 0.0};

  bool preemt_requested_;
  static constexpr char LOGNAME[]{ "task planner" };

  TaskParameters parameters; 
  

  ros::NodeHandle pnh_;
};
#endif
