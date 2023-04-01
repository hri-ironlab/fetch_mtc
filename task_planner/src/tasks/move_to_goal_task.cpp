#include <tasks/move_to_goal_task.h>

constexpr char LOGNAME[] = "move to task";

MoveToGoalTask::MoveToGoalTask(const std::string& task_name) : task_name_(task_name)
{
  task_.reset();
  task_.reset(new moveit::task_constructor::Task(task_name_));
  task_->loadRobotModel();
  current_state_stage_ = nullptr;
}

bool MoveToGoalTask::init(const TaskParameters& parameters)
{
  ROS_INFO_NAMED(LOGNAME, "Initializing move to pipeline");
  // Create new Task and call it pick_place_task with task_name_name
  // Task is the root of a tree of stages, default serial container wraps all stages
  Task& t = *task_;

  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Set task properties, names used for the specific arm group by robot
  // Set task properties
  t.setProperty("group", parameters.arm_group_name_);
  t.setProperty("eef", parameters.eef_name_);
  t.setProperty("hand", parameters.hand_group_name_);
  t.setProperty("ik_frame", parameters.hand_frame_);
  
  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");
    t.add(std::move(_current_state));
  }


  if(parameters.use_joint_positions_){
    for (int i = 0; i < parameters.robot_states_.size(); i++)
    {
      std::string goal = "Goal " + std::to_string(i);
      auto stage = std::make_unique<stages::MoveTo>(goal, sampling_planner);
      stage->setGroup(parameters.arm_group_name_);
      stage->setGoal(parameters.robot_states_[i]);
      t.add(std::move(stage));
    }
  }
  else{
    for (int i = 0; i < parameters.hand_poses_.size(); i++)
    {
      std::string goal = "Goal " + std::to_string(i);
      auto stage = std::make_unique<stages::MoveTo>(goal, sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });

      stage->setGoal(parameters.hand_poses_[i]);
      t.add(std::move(stage));
    }
  }

  try
  {
    t.init();
  }
  catch (InitStageException& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
    return false;
  }

  return true;
}

bool MoveToGoalTask::plan()
{
  ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
  try
  {
    task_->plan(1);
  }
  catch (InitStageException& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
    return false;
  }
  if (task_->numSolutions() == 0)
  {
    ros::waitForShutdown(); 
    ROS_ERROR_NAMED(LOGNAME, "Planning failed");
    return false;
  }
  return true;
}

bool MoveToGoalTask::preempt()
{
  task_->preempt();
  return true;
}

void MoveToGoalTask::getSolutionMsg(moveit_task_constructor_msgs::Solution& solution)
{
  task_->solutions().front()->fillMessage(solution);
}

bool MoveToGoalTask::execute()
{
  ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
  moveit_msgs::MoveItErrorCodes execute_result;
  task_->introspection().publishSolution(*task_->solutions().front());
  execute_result = task_->execute(*task_->solutions().front());
  // // If you want to inspect the goal message, use this instead:
  // actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
  // execute("execute_task_solution", true); execute.waitForServer();
  // moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
  // task_->solutions().front()->fillMessage(execute_goal.solution);
  // execute.sendGoalAndWait(execute_goal);
  // execute_result = execute.getResult()->error_code;

  if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_result.val);
    return false;
  }

  return true;
}