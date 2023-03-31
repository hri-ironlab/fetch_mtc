#include <task_planner/task_planner.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

constexpr char LOGNAME[] = "moveit_task_constructor";
constexpr char TaskPlanner::LOGNAME[];

void TaskPlanner::loadParameters(const ros::NodeHandle& pnh_)
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
  std::string surface_link_two;
  // Target object
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link", surface_link);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link_two", surface_link_two);
  parameters.support_surfaces_ = { surface_link, surface_link_two };
  // Pick/Place metrics
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", parameters.approach_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", parameters.approach_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", parameters.lift_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", parameters.lift_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset", parameters.place_surface_offset_);
  
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "pick_object", parameters.object_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_pose", parameters.place_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_offset", parameters.object_offset_);
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

  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

TaskPlanner::TaskPlanner()
{
  //initializeServer();
}

void TaskPlanner::initializeServer()
{
  pick_place_server_ = std::make_unique<PickPlaceServer>(
      pnh_, "pick_place_server", [this](const auto& goal) { executePickPlaceCallback(goal); }, false);
  pick_place_server_->registerPreemptCallback([this] { preemptPickPlaceCallback(); });
  pick_place_server_->start();
}

void TaskPlanner::TestPickPlace()
{
  parameters.task_type_ = task_planner::PlanPickPlaceGoal::PICK_AND_PLACE;

  ROS_INFO_STREAM(parameters.place_pose_ << "\n" << parameters.object_name_);

  if(parameters.task_type_ == task_planner::PlanPickPlaceGoal::PICK_AND_PLACE)
  {  
    auto pickplace = std::make_unique<PickPlaceTask>("pick_and_place");
    if (!pickplace->init(parameters))
    {
      ROS_INFO_NAMED(LOGNAME, "Initialization failed");
      return;
    }

    if (pickplace->plan())
    {
        ROS_INFO_NAMED(LOGNAME, "Planning succeded");
        moveit_task_constructor_msgs::ExecuteTaskSolutionActionResult msg;
        moveit_task_constructor_msgs::ExecuteTaskSolutionResult result; 
        result.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS; 
        msg.result = result; 
        plan_status.publish(msg);
        ros::Duration(3.0).sleep();

        if (pickplace->execute())
        {
          ROS_INFO_NAMED(LOGNAME, "Execution complete");
        }
        else
        {
           ROS_INFO_NAMED(LOGNAME, "Execution failed");
        }
    }
    else
    {
      moveit_task_constructor_msgs::ExecuteTaskSolutionActionResult msg;
      moveit_task_constructor_msgs::ExecuteTaskSolutionResult result; 
      result.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED; 
      msg.result = result; 
      plan_status.publish(msg);
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }
  }
}

void TaskPlanner::planRobotActionCallback(const task_planner::PickPlaceConstPtr& msg)
{
  ROS_INFO("Message recieved");

  parameters.object_name_ = msg->object_name;
  // If accepting parameterized frame_transform
  parameters.place_pose_ = msg->place_pose;

  if(msg->task_type ==  task_planner::PickPlace::PICK_AND_PLACE)
  {
    parameters.task_type_ = task_planner::PlanPickPlaceGoal::PICK_AND_PLACE;
    auto pickplace = std::make_unique<PickPlaceTask>("pick and place");
    if (!pickplace->init(parameters))
    {
      ROS_INFO_NAMED(LOGNAME, "Initialization failed");
      return;
    }

    if (pickplace->plan())
    {
      ROS_INFO_NAMED(LOGNAME, "Planning succeded");
      if(msg->execute){
        if (pickplace->execute())
        {
          ROS_INFO_NAMED(LOGNAME, "Execution complete");
        }
        else
        {
          ROS_INFO_NAMED(LOGNAME, "Execution failed");
        }
      }
    }
    else{
      moveit_task_constructor_msgs::ExecuteTaskSolutionActionResult msg;
      moveit_task_constructor_msgs::ExecuteTaskSolutionResult result; 
      result.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED; 
      msg.result = result; 
      plan_status.publish(msg);
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }
  }
}

void TaskPlanner::executePickPlaceCallback(const task_planner::PlanPickPlaceGoalConstPtr& goal)
{
  pick_place_server_->setSucceeded();
}

void TaskPlanner::preemptPickPlaceCallback()
{
  preemt_requested_ = true;
  pick_place_server_->setPreempted();
}
