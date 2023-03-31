#include <iostream>
#include "ros/ros.h"
#include "scene_handler/scene_handler.h"

int
main( int argc,char* argv[] ){
	ros::init( argc, argv, "scene_handler_node" );
	ros::NodeHandle node_handle;

	SceneHandler sceneHandler; 

	// Uncomment if you want to send object msgs to Unity to display
	// ros::Subscriber add_objects_subscriber = node_handle.subscribe( "/unity/scene_objects", 1, &SceneHandler::sceneObjectsCallback,  &sceneHandler);

	ROS_INFO("Scene handler begin");

	ros::Duration(5.0).sleep();
	ros::Rate loop_rate(40); 
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep(); 
	}

	return 0;
}
