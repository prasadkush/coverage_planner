// Code written by Kush Prasad //
// email: prasad.kush@gmail.com //

#include <ros/ros.h>
#include <coverage_planner/coverage_planner.h>
#include <tf/transform_broadcaster.h>
#include <csignal>

// This code instantiates an object of the coverage_planner class
// The constructor of the coverage_planner class calls the input_polygon() function from which ...
// ... compute_plan() function is called which computes the coverage plan

void spinthread()
{
	ros::spin();
}

void signalHandler(int sig)
{
	ros::Rate r(2);
	ros::shutdown();
	exit(sig);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "planner");
	tf::TransformListener tf_(ros::Duration(10));
	ros::NodeHandle n;
	ros::Rate r(10);
	signal(SIGINT, signalHandler);
	coverage_planner planner(n);
	boost::thread th(spinthread);
	planner.th_input->join();
	while (1)
	{
		r.sleep();
		if (planner.exit_condition)
		{
			coverage_planner::exit_gracefully(&planner);	
		}
	}

	//ros::spin();
	return 0;
}


