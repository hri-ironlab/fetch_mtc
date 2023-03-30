#include <iostream>
#include "scene_handler/scene_handler.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "setup scene");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  auto sceneHandler = std::make_unique<SceneHandler>();
  sceneHandler->setupStartScene(pnh);
  // Keep introspection alive
  // ros::waitForShutdown();
  return 0;
}