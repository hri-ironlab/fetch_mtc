#ifndef TASKBASE_H
#define TASKBASE_H

#include <task_planner/task_parameters.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

class TaskBase
{
public:
  TaskBase();
  virtual ~TaskBase(){};
  virtual bool init(const TaskParameters& parameters) = 0;
  virtual bool plan() = 0;
  virtual bool preempt() = 0;
  virtual bool execute() = 0;
  virtual void getSolutionMsg(moveit_task_constructor_msgs::Solution& solution) = 0;
};

#endif