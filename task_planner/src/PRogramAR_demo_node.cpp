#include <iostream>
#include "task_planner/task_planner.h"
#include "scene_handler/scene_handler.h"
#include "tasks/pick_place_task.h"
#include "tasks/move_to_goal_task.h"
#include "tasks/open_close_gripper_task.h"
#include <task_planner/PlanPickPlaceAction.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <task_planner/task_parameters.h>

constexpr char LOGNAME[] = "PRogramAR demo node";

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "PRogramAR demo node");
  ros::NodeHandle nh, pnh("~");

  auto sceneHandler = std::make_unique<SceneHandler>();
  auto taskPlanner = std::make_unique<TaskPlanner>(); 
  taskPlanner->loadParameters(pnh);

  // Scene objects from unity are added to the planning scene
	ros::Subscriber add_objects_subscriber = nh.subscribe( "/unity/scene_objects", 1, &SceneHandler::sceneObjectsCallback,  &(*sceneHandler));

  // Recieves the action plan and calculates the pick and place sequence
  ros::Subscriber plan_robot_action_subscriber = nh.subscribe( "/unity/query_planner", 1, &TaskPlanner::planRobotActionCallback, &(*taskPlanner));

  // Sends back wheter we can execute the task or not with the plan solution
  taskPlanner->plan_status = nh.advertise<moveit_task_constructor_msgs::ExecuteTaskSolutionActionResult>("/execute_task_solution/result", 1, true);

	ros::Duration(5.0).sleep();
	ros::Rate loop_rate(40); 
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep(); 
	}

  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}
