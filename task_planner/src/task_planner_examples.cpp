#include <iostream>
#include "task_planner/task_planner.h"
#include "scene_handler/scene_handler.h"
#include "tasks/pick_place_task.h"
#include "tasks/move_to_goal_task.h"
#include "tasks/open_close_gripper_task.h"
#include <task_planner/PlanPickPlaceAction.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <task_planner/task_parameters.h>

constexpr char LOGNAME[] = "task planner examples";

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
  
  int transform_orientation;
  std::vector<double> diagonal_frame_transform {0.02, 0, 0.0, 0, 3.9275, 0.0};  
  std::vector<double> horizontal_frame_transform {0.02, 0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> vertical_frame_transform {0.02, 0, 0.0,  0, 4.713, 0.0};
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "transform_orientation", transform_orientation);

  // Should do this better at some point
  if(transform_orientation == 0){
    rosparam_shortcuts::convertDoublesToEigen(LOGNAME, horizontal_frame_transform ,parameters.grasp_frame_transform_);
  }
  else if(transform_orientation == 1){
    rosparam_shortcuts::convertDoublesToEigen(LOGNAME, vertical_frame_transform ,parameters.grasp_frame_transform_);
  }
  else{
    rosparam_shortcuts::convertDoublesToEigen(LOGNAME, diagonal_frame_transform ,parameters.grasp_frame_transform_);
  }

  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "pick_object", parameters.object_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_pose", parameters.place_pose_);

  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "task_planner_examples");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  TaskParameters parameters;
  loadParameters(pnh, parameters);

  /// ***************************** Link following hand wave example  ***************************** ///
  geometry_msgs::PoseStamped hand_wave_start_pose;
  hand_wave_start_pose.header.frame_id = parameters.base_frame_;
  hand_wave_start_pose.pose.position.x = 0.042;
  hand_wave_start_pose.pose.position.y = 0.384;
  hand_wave_start_pose.pose.position.z = 1.626;
  hand_wave_start_pose.pose.orientation.y = 0.7068907;
  hand_wave_start_pose.pose.orientation.w = -0.7073228;
  
  geometry_msgs::PoseStamped hand_wave_end_pose;
  hand_wave_end_pose.header.frame_id = parameters.base_frame_;
  hand_wave_end_pose.pose.position.x = 0.047;
  hand_wave_end_pose.pose.position.y = 0.645;
  hand_wave_end_pose.pose.position.z = 1.622;
  hand_wave_end_pose.pose.orientation.y = 0.7068907;
  hand_wave_end_pose.pose.orientation.w = -0.7073228;

  std::vector<geometry_msgs::PoseStamped> hand_poses
  {
    hand_wave_start_pose,
    hand_wave_end_pose,
    hand_wave_start_pose,
    hand_wave_end_pose,
    hand_wave_start_pose,
    hand_wave_end_pose,
    hand_wave_start_pose,
    hand_wave_end_pose   
  };

  ROS_INFO("************** Demonstrating MoveToGoalTask Hand Wave Example **************");
  /// ***************************** Using Goal control example ***************************** ///
  auto wave = std::make_unique<MoveToGoalTask>("Move a couple times");
  parameters.use_joint_positions_ = false;
  parameters.hand_poses_ = hand_poses;
  if (!wave->init(parameters))
  {
    ROS_INFO_NAMED(LOGNAME, "Initialization failed");
    return 1;
  }
  if (wave->plan())
  {
    ROS_INFO_NAMED(LOGNAME, "Planning succeded");
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }

  if (wave->execute())
  {
    ROS_INFO_NAMED(LOGNAME, "Execution complete");
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Execution failed");
  }

  /// ***************************** Joint position following disco dance example  ***************************** ///
  std::vector<std::string> joint_names
  {
    "torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
    "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"
  };

  std::vector<std::vector<double>> disco_poses
  {
    {0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0},
    {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
    {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
    {0.385, -1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0},
    {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
    {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
    {0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0}
  };

  for(int i = 0; i<disco_poses.size(); i++){
    moveit_msgs::RobotState robot_state;
    for(int j = 0; j<disco_poses[i].size(); j++){
      robot_state.joint_state.name.push_back(joint_names[j]);
      robot_state.joint_state.position.push_back(disco_poses[i][j]);
    }
    robot_state.is_diff = true;
    parameters.robot_states_.push_back(robot_state);
  }

  ROS_INFO("************** Demonstrating MoveToGoalTask Disco dance **************");
  /// ***************************** Using Goal control example ***************************** ///
  auto disco = std::make_unique<MoveToGoalTask>("Move a couple times");
  parameters.use_joint_positions_ = true; 
  if (!disco->init(parameters))
  {
    ROS_INFO_NAMED(LOGNAME, "Initialization failed");
    return 1;
  }

  if (disco->plan())
  {
    ROS_INFO_NAMED(LOGNAME, "Planning succeded");
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }

  if (disco->execute())
  {
    ROS_INFO_NAMED(LOGNAME, "Execution complete");
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Execution failed");
  }

  /// ***************************** Scene Setup  ***************************** ///
  auto sceneHandler = std::make_unique<SceneHandler>();
  sceneHandler->setupStartScene(pnh);

  /// ***************************** Pick object example ***************************** ///
  ROS_INFO("************** Demonstrating PickPlaceTask Pick Object **************");
  auto pick = std::make_unique<PickPlaceTask>("Pick object");
  parameters.task_type_ = task_planner::PlanPickPlaceGoal::PICK_ONLY;
  
  if (!pick->init(parameters))
  {
    ROS_INFO_NAMED(LOGNAME, "Initialization failed");
    return 1;
  }

  if (pick->plan())
  {
    ROS_INFO_NAMED(LOGNAME, "Planning succeded");
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }

  if (pick->execute())
  {
    ROS_INFO_NAMED(LOGNAME, "Execution complete");
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Execution failed");
  }

  /// ***************************** Place object example ***************************** ///
  ROS_INFO("************** Demonstrating PickPlaceTask Place Object **************");
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
  }

  /// ***************************** Open gripper example ***************************** ///
  ROS_INFO("************** Demonstrating OpenCloseGripper Task Close Gripper **************");
  auto close = std::make_unique<OpenCloseGripperTask>("Close gripper");
  parameters.open_hand_ = false;

  if (!close->init(parameters))
  {
    ROS_INFO_NAMED(LOGNAME, "Initialization failed");
    return 1;
  }

  if (close->plan())
  {
    ROS_INFO_NAMED(LOGNAME, "Planning succeded");
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }

  if (close->execute())
  {
    ROS_INFO_NAMED(LOGNAME, "Execution complete");
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Execution failed");
  }

  /// ***************************** Open gripper example ***************************** ///
  ROS_INFO("************** Demonstrating OpenCloseGripper Task Open Gripper **************");
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


  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}
