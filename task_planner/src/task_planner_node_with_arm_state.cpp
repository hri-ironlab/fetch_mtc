#include <iostream>
#include "task_planner/task_planner.h"
#include "scene_handler/scene_handler.h"
#include "tasks/pick_place_task.h"
#include "tasks/move_to_waypoints_task.h"
#include "tasks/open_close_gripper_task.h"
#include <task_planner/PlanPickPlaceAction.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <task_planner/task_parameters.h>

constexpr char LOGNAME[] = "task planner node";

void loadParameters(const ros::NodeHandle& pnh_, TaskParameters& parameters)
{
  /****************************************************
   *                                                  *
   *               Load Parameters                    *
   *                                                  *
   ***************************************************/
  ROS_INFO_NAMED(LOGNAME, "Loading task parameters");

  // Planning group properties
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_group_name", parameters.arm_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_group_name", parameters.hand_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "eef_name", parameters.eef_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_frame", parameters.hand_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "base_frame", parameters.base_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "vertical_frame_transform", parameters.grasp_frame_transform_);

  // Predefined pose targets
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_open_pose", parameters.hand_open_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_close_pose", parameters.hand_close_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_home_pose", parameters.arm_home_pose_);

  std::string surface_link;
  // Target object
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link", surface_link);
  parameters.support_surfaces_ = { surface_link };
  // Pick/Place metrics
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", parameters.approach_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", parameters.approach_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", parameters.lift_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", parameters.lift_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset", parameters.place_surface_offset_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "task planner node");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  auto sceneHandler = std::make_unique<SceneHandler>();
  sceneHandler->setupStartScene(pnh);

  std::string box_name = "box";
  geometry_msgs::Pose box_place_pose;
  box_place_pose.position.x = 0.5;
  box_place_pose.position.y = -.25;
  box_place_pose.position.z = .6;

  box_place_pose.orientation.x = 0.0;
  box_place_pose.orientation.y = 0.0;
  box_place_pose.orientation.z = 0.0;
  box_place_pose.orientation.w = 1.0;

  std::string box_name2 = "box2";
  geometry_msgs::Pose box2_place_pose;
  box2_place_pose.position.x = 0.7;
  box2_place_pose.position.y = -.1;
  box2_place_pose.position.z = .6;

  box2_place_pose.orientation.x = 0.0;
  box2_place_pose.orientation.y = 0.0;
  box2_place_pose.orientation.z = 0.0;
  box2_place_pose.orientation.w = 1.0;

  std::string cylinder_name = "cylinder";
  geometry_msgs::Pose cylinder_place_pose;
  cylinder_place_pose.position.x = 0.6;
  cylinder_place_pose.position.y = -0.25;
  cylinder_place_pose.position.z = .6;

  cylinder_place_pose.orientation.x = 0.0;
  cylinder_place_pose.orientation.y = 0.0;
  cylinder_place_pose.orientation.z = 0.0;
  cylinder_place_pose.orientation.w = 1.0;

  TaskParameters parameters;
  loadParameters(pnh, parameters);

  parameters.task_type_ = task_planner::PlanPickPlaceGoal::PICK_AND_PLACE;
  parameters.object_name_ = "box2";
  parameters.place_pose_ = box2_place_pose;

  /* std::vector<moveit_msgs::RobotState> robot_states;
   moveit_msgs::RobotState robot_state;

   robot_state.joint_state.name.push_back("panda_joint1");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint2");
   robot_state.joint_state.position.push_back(-0.785);

   robot_state.joint_state.name.push_back("panda_joint3");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint4");
   robot_state.joint_state.position.push_back(-2.356);

   robot_state.joint_state.name.push_back("panda_joint5");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint6");
   robot_state.joint_state.position.push_back(1.571);

   robot_state.joint_state.name.push_back("panda_joint7");
   robot_state.joint_state.position.push_back(0.785);

   robot_state.is_diff = true;

   parameters.robot_states_.push_back(robot_state);

   robot_state.joint_state.name.push_back("panda_joint1");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint2");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint3");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint4");
   robot_state.joint_state.position.push_back(-2.356);

   robot_state.joint_state.name.push_back("panda_joint5");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint6");
   robot_state.joint_state.position.push_back(1.571);

   robot_state.joint_state.name.push_back("panda_joint7");
   robot_state.joint_state.position.push_back(0.785);

   robot_state.is_diff = true;
   parameters.robot_states_.push_back(robot_state);

   robot_state.joint_state.name.push_back("panda_joint1");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint2");
   robot_state.joint_state.position.push_back(-0.785);

   robot_state.joint_state.name.push_back("panda_joint3");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint4");
   robot_state.joint_state.position.push_back(-2.356);

   robot_state.joint_state.name.push_back("panda_joint5");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint6");
   robot_state.joint_state.position.push_back(1.571);

   robot_state.joint_state.name.push_back("panda_joint7");
   robot_state.joint_state.position.push_back(0.785);

   robot_state.is_diff = true;

   parameters.robot_states_.push_back(robot_state);

   robot_state.joint_state.name.push_back("panda_joint1");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint2");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint3");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint4");
   robot_state.joint_state.position.push_back(-2.356);

   robot_state.joint_state.name.push_back("panda_joint5");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint6");
   robot_state.joint_state.position.push_back(1.571);

   robot_state.joint_state.name.push_back("panda_joint7");
   robot_state.joint_state.position.push_back(0.785);

   robot_state.is_diff = true;
   parameters.robot_states_.push_back(robot_state);

   robot_state.joint_state.name.push_back("panda_joint1");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint2");
   robot_state.joint_state.position.push_back(-0.785);

   robot_state.joint_state.name.push_back("panda_joint3");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint4");
   robot_state.joint_state.position.push_back(-2.356);

   robot_state.joint_state.name.push_back("panda_joint5");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint6");
   robot_state.joint_state.position.push_back(1.571);

   robot_state.joint_state.name.push_back("panda_joint7");
   robot_state.joint_state.position.push_back(0.785);

   robot_state.is_diff = true;

   parameters.robot_states_.push_back(robot_state);

   robot_state.joint_state.name.push_back("panda_joint1");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint2");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint3");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint4");
   robot_state.joint_state.position.push_back(-2.356);

   robot_state.joint_state.name.push_back("panda_joint5");
   robot_state.joint_state.position.push_back(0.0);

   robot_state.joint_state.name.push_back("panda_joint6");
   robot_state.joint_state.position.push_back(1.571);

   robot_state.joint_state.name.push_back("panda_joint7");
   robot_state.joint_state.position.push_back(0.785);

   robot_state.is_diff = true;
   parameters.robot_states_.push_back(robot_state);*/
  /*
    auto move = std::make_unique<MoveToWaypointsTask>("Move a couple times");
    if (!move->init(parameters))
    {
      ROS_INFO_NAMED(LOGNAME, "Initialization failed");
      return 1;
    }

    if (move->plan())
    {
      ROS_INFO_NAMED(LOGNAME, "Planning succeded");
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }

    if (move->execute())
    {
      ROS_INFO_NAMED(LOGNAME, "Execution complete");
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Execution failed");
    }
  */
  auto pick = std::make_unique<PickPlaceTask>("Pick object");
  if (!pick->init(parameters))
  {
    ROS_INFO_NAMED(LOGNAME, "Initialization failed");
    return 1;
  }

  if (pick->plan())
  {
	moveit_task_constructor_msgs::Solution solution;
	pick.getSolutionMsg(solution);
	publish
    ROS_INFO_NAMED(LOGNAME, "Planning succeded");
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }

//  if (pick->execute())
//  {
 //   ROS_INFO_NAMED(LOGNAME, "Execution complete");
 // }
 // else
 // {
 //   ROS_INFO_NAMED(LOGNAME, "Execution failed");
 // }
  /*
    auto open = std::make_unique<OpenCloseGripperTask>("Open gripper");
    parameters.open_hand_ = true;

    if (!open->init(parameters))
    {
      ROS_INFO_NAMED(LOGNAME, "Initialization failed");
      return 1;
    }

    if (open->plan())
    {
      ROS_INFO_NAMED(LOGNAME, "Planning succeded");
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }

    if (open->execute())
    {
      ROS_INFO_NAMED(LOGNAME, "Execution complete");
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Execution failed");
    }

    auto place = std::make_unique<PickPlaceTask>("Place Object");
    parameters.task_type_ = task_planner::PlanPickPlaceGoal::PLACE_ONLY;

    if (!place->init(parameters))
    {
      ROS_INFO_NAMED(LOGNAME, "Initialization failed");
      return 1;
    }

    if (place->plan())
    {
      ROS_INFO_NAMED(LOGNAME, "Planning succeded");
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }

    if (place->execute())
    {
      ROS_INFO_NAMED(LOGNAME, "Execution complete");
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Execution failed");
    }*/

  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}
