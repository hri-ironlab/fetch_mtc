#include <task_planner/task_planner.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

constexpr char LOGNAME[] = "moveit_task_constructor";
constexpr char TaskPlanner::LOGNAME[];

TaskPlanner::TaskPlanner(const ros::NodeHandle& pnh) : pnh_(pnh)
{
  loadParameters();
  loadPlanners();
  // initializeServer();
}

void TaskPlanner::loadParameters()
{
  /****************************************************
   *                                                  *
   *               Load Parameters                    *
   *                                                  *
   ***************************************************/
  ROS_INFO_NAMED(LOGNAME, "Loading task parameters");

  // Planning group properties
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_group_name", arm_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_group_name", hand_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "eef_name", eef_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_frame", hand_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "world_frame", world_frame_);

  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "diagonal_frame_transform", diagonal_frame_transform_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "downward_frame_transform", downward_frame_transform_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "vertical_frame_transform", vertical_frame_transform_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "horizontal_frame_transform", horizontal_frame_transform_);
  // Predefined pose targets
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_open_pose", hand_open_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_close_pose", hand_close_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_home_pose", arm_home_pose_);

  // Target object
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link", surface_link_);
  support_surfaces_ = { surface_link_ };

  // Pick/Place metrics
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", approach_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", approach_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", lift_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", lift_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset", place_surface_offset_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void TaskPlanner::loadPlanners()
{
  sampling_planner_ = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner_->setProperty("goal_joint_tolerance", 1e-5);

  cartesian_solver_ = std::make_shared<solvers::CartesianPath>();
  cartesian_solver_->setMaxVelocityScaling(1.0);
  cartesian_solver_->setMaxAccelerationScaling(1.0);
  cartesian_solver_->setStepSize(.01);
}

Eigen::Isometry3d TaskPlanner::getGraspOrientation(TaskPlanner::GraspOrientation grasp_orientation)
{
  switch (grasp_orientation)
  {
    case GraspOrientation::downward:
      return downward_frame_transform_;
      break;
    case GraspOrientation::diagonal:
      return diagonal_frame_transform_;
      break;
    case GraspOrientation::vertical:
      return vertical_frame_transform_;
      break;
    default:
      return horizontal_frame_transform_;
  }
}
/*
void TaskPlanner::initializeServer(){
  pick_place_server_ = std::make_unique<PickPlaceServer>(pnh_, "pick_place_server", [this](const auto& goal){
executePickPlaceCallback(goal); }, false); pick_place_server_->registerPreemptCallback([this]
{preemptPickPlaceCallback(); }); pick_place_server_->start();
}

void TaskPlanner::executePickPlaceCallback(const moveit_task_constructor_msgs::PlanPickPlaceGoalConstPtr& goal){
  ROS_INFO("%s", goal->hand_group_name);
  pick_place_server_->setSucceeded();
}

void TaskPlanner::preemptPickPlaceCallback()
{
  preemt_requested_ = true;
  pick_place_server_->setPreempted();
}
*/

std::unique_ptr<stages::PredicateFilter> TaskPlanner::setCurrentStatePredicate(Stage*& current_state_ptr,
                                                                               const std::string object)
{
  // Initialize currentstate stage with name current state. This current_state holds a reference to the robot model,
  // scene and properties
  auto current_state = std::make_unique<stages::CurrentState>("current state");

  // Verify that object is not attached
  auto applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
  applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
    if (s.start()->scene()->getCurrentState().hasAttachedBody(object))
    {
      comment = "object with id '" + object + "is already attached and cannot be picked ";
      return false;
    }
    return true;
  });

  current_state_ptr = applicability_filter.get();
  return applicability_filter;
}

// std::unique_ptr<stages::CurrentState> TaskPlanner::SetCurrentState()

std::unique_ptr<stages::MoveTo> TaskPlanner::openHand()
{
  auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner_);
  stage->setGroup(hand_group_name_);
  stage->setGoal(hand_open_pose_);
  return stage;
}

std::unique_ptr<stages::Connect> TaskPlanner::connectStages(std::string stage_name)
{
  auto stage = std::make_unique<stages::Connect>(
      stage_name, stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner_ } });
  stage->setTimeout(5.0);
  stage->properties().configureInitFrom(Stage::PARENT);
  return stage;
}

std::unique_ptr<SerialContainer> TaskPlanner::pickObject(Task& t, const std::string object,
                                                         const Eigen::Isometry3d& grasp_frame_transform,
                                                         Stage*& current_state_ptr, Stage*& attach_object_stage)
{
  auto grasp = std::make_unique<SerialContainer>("pick object");
  t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
  grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

  /****************************************************
---- *               Approach Object                    *
   ***************************************************/
  {
    auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_solver_);
    stage->properties().set("marker_ns", "approach_object");
    stage->properties().set("link", hand_frame_);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

    // Set hand forward direction
    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = world_frame_;
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }

  /****************************************************
---- *               Generate Grasp Pose                *
   ***************************************************/
  {
    // Sample grasp pose
    auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setPreGraspPose(hand_open_pose_);
    stage->setObject(object);
    stage->setAngleDelta(M_PI / 12);
    stage->setMonitoredStage(current_state_ptr);  // Hook into current state

    // Compute IK
    auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
    wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(wrapper));
  }

  /****************************************************
---- *               Allow Collision (hand object)   *
   ***************************************************/
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions(
        object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
        true);
    grasp->insert(std::move(stage));
  }

  /****************************************************
---- *               Close Hand                      *
   ***************************************************/
  {
    auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner_);
    stage->setGroup(hand_group_name_);
    stage->setGoal(hand_close_pose_);
    grasp->insert(std::move(stage));
  }

  /****************************************************
.... *               Attach Object                      *
   ***************************************************/
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
    stage->attachObject(object, hand_frame_);
    attach_object_stage = stage.get();
    grasp->insert(std::move(stage));
  }

  /****************************************************
.... *               Allow collision (object support)   *
   ***************************************************/
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
    stage->allowCollisions({ object }, support_surfaces_, true);
    grasp->insert(std::move(stage));
  }

  /****************************************************
.... *               Lift object                        *
   ***************************************************/
  {
    auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_solver_);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
    stage->setIKFrame(hand_frame_);
    stage->properties().set("marker_ns", "lift_object");

    // Set upward direction
    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = world_frame_;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }

  /****************************************************
.... *               Forbid collision (object support)  *
   ***************************************************/
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
    stage->allowCollisions({ object }, support_surfaces_, false);
    grasp->insert(std::move(stage));
  }

  // Add grasp container to task
  return grasp;
}

std::unique_ptr<SerialContainer> TaskPlanner::placeObject(Task& t, const std::string object,
                                                          const Eigen::Isometry3d& grasp_frame_transform,
                                                          const geometry_msgs::Pose& place_pose,
                                                          Stage*& current_state_ptr, Stage*& attach_object_stage)
{
  auto place = std::make_unique<SerialContainer>("place object");
  t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
  place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

  /******************************************************
---- *          Lower Object                              *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_solver_);
    stage->properties().set("marker_ns", "lower_object");
    stage->properties().set("link", hand_frame_);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setMinMaxDistance(.03, .13);

    // Set downward direction
    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = world_frame_;
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    place->insert(std::move(stage));
  }

  /******************************************************
---- *          Generate Place Pose                       *
   *****************************************************/
  {
    // Generate Place Pose
    auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
    stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
    stage->properties().set("marker_ns", "place_pose");
    stage->setObject(object);

    // Get object from planning scene
    collision_object_ = psi.getObjects({ object })[object];

    // Set target pose
    geometry_msgs::PoseStamped p;
    p.header.frame_id = collision_object_.header.frame_id;  // object_reference_frame was used before
    p.pose = place_pose;

    // Take half the objects height and add that to the z position to properly place the object
    if (collision_object_.primitives[0].type == shape_msgs::SolidPrimitive::BOX)
    {
      p.pose.position.z += 0.5 * collision_object_.primitives[0].dimensions[2] + place_surface_offset_;
    }
    else
    {
      p.pose.position.z += 0.5 * collision_object_.primitives[0].dimensions[0] + place_surface_offset_;
    }
    stage->setPose(p);
    stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

    // Compute IK
    auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(5);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
    wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
    place->insert(std::move(wrapper));
  }

  /******************************************************
---- *          Open Hand                              *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner_);
    stage->setGroup(hand_group_name_);
    stage->setGoal(hand_open_pose_);
    place->insert(std::move(stage));
  }

  /******************************************************
---- *          Forbid collision (hand, object)        *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
    stage->allowCollisions(object, *t.getRobotModel()->getJointModelGroup(hand_group_name_), false);
    place->insert(std::move(stage));
  }

  /******************************************************
---- *          Detach Object                             *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
    stage->detachObject(object, hand_frame_);
    place->insert(std::move(stage));
  }

  /******************************************************
---- *          Retreat Motion                            *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_solver_);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setMinMaxDistance(.12, .25);
    stage->setIKFrame(hand_frame_);
    stage->properties().set("marker_ns", "retreat");
    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = hand_frame_;
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    place->insert(std::move(stage));
  }

  // Add place container to task
  return place;
}

std::unique_ptr<stages::MoveRelative> TaskPlanner::rotateArm(std::string& stage_name)
{
  auto stage = std::make_unique<stages::MoveRelative>(stage_name, cartesian_solver_);
  stage->setGroup(arm_group_name_);
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "world";
  twist.twist.angular.z = M_PI / 4.;
  stage->setDirection(twist);
  return (stage);
}

std::unique_ptr<stages::MoveTo> TaskPlanner::moveHome()
{
  auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner_);
  stage->properties().configureInitFrom(Stage::PARENT, { "group" });
  stage->setGoal(arm_home_pose_);
  stage->restrictDirection(stages::MoveTo::FORWARD);
  return stage;
}

bool TaskPlanner::initPickPlaceTask(const std::string& object_name, TaskPlanner::GraspOrientation grasp_orientation,
                                    const geometry_msgs::Pose& place_pose)
{
  ROS_INFO_NAMED(LOGNAME, "Initializing pick and place object pipeline");
  const std::string object = object_name;

  // Reset ROS introspection before constructing the new object
  // TODO(v4hn): global storage for Introspection services to enable one-liner
  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  // Create new Task and call it pick_place_task with task_name_name
  // Task is the root of a tree of stages, default serial container wraps all stages

  Task& t = *task_;
  t.stages()->setName("Pick and Place Task");
  t.loadRobotModel();

  // Set task properties, names used for the specific arm group by robot
  t.setProperty("group", arm_group_name_);
  t.setProperty("eef", eef_name_);
  t.setProperty("hand", hand_group_name_);
  t.setProperty("hand_grasping_frame", hand_frame_);
  t.setProperty("ik_frame", hand_frame_);

  Eigen::Isometry3d grasp_frame_transform = getGraspOrientation(grasp_orientation);

  // Current State
  Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  t.add(setCurrentStatePredicate(current_state_ptr, object));

  // Open Hand
  t.add(openHand());

  // Move to Pick
  t.add(connectStages("move to pick"));

  // Pick Object
  Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
  t.add(pickObject(t, object, grasp_frame_transform, current_state_ptr, attach_object_stage));

  // Move to Place
  t.add(connectStages("move to place"));

  // Place Object
  t.add(placeObject(t, object, grasp_frame_transform, place_pose, current_state_ptr, attach_object_stage));

  // Move to Home
  t.add(moveHome());

  // prepare Task structure for planning
  return taskInit(t);
}

bool TaskPlanner::taskInit(Task& t)
{
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

bool TaskPlanner::plan()
{
  ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
  int max_solutions = pnh_.param<int>("max_solutions", 5);

  return static_cast<bool>(task_->plan(max_solutions));
}

bool TaskPlanner::execute()
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
