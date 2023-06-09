#include <iostream>
#include "task_planner/task_planner.h"
#include "scene_handler/scene_handler.h"
#include "tasks/pick_place_task.h"
#include "tasks/move_to_goal_task.h"
#include "tasks/open_close_gripper_task.h"
#include <task_planner/PlanPickPlaceAction.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <task_planner/task_parameters.h>

constexpr char LOGNAME[] = "task planner node";

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "testpickplace");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration(1.0).sleep();

  auto taskPlanner = std::make_unique<TaskPlanner>(); 
  taskPlanner->loadParameters(pnh);

  taskPlanner->plan_status = nh.advertise<moveit_task_constructor_msgs::ExecuteTaskSolutionActionResult>("/execute_task_solution/result", 1, true);
  taskPlanner->TestPickPlace(); 

  // Keep introspection alive
  //ros::waitForShutdown();
  return 0;
}
