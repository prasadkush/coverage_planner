#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>
#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <coverage_planner/environment.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <sstream>


// This code visualizes polygons, obstacles and paths. It takes input from functions in coverage_planner.cpp

class visualize
{
public:

	visualize(ros::NodeHandle& nh);

	~visualize();

	void initialize_marker_variables_polygon(bool v, const environment& env);

	void initialize_marker_variables_obstacles(int c, const environment& env);

	void initialize_marker_variables_path(const vector<geometry_msgs::Point>& plan);

	void draw_polygon();

	void draw_obstacles();

	void vis_path();

	bool marker_initialize, obs_marker_initialized, path_marker_initialized;
	bool draw_polygons_running, draw_obstacles_running, vis_path_running;

	ros::Publisher marker_pub;
	ros::Publisher marker_pub_obs;
	ros::Publisher marker_pub_path;

	ros::NodeHandle n;

	boost::thread* th_draw_polygons;
	boost::thread* th_draw_obstacles;
	boost::thread* th_vis_path;

private:

	visualization_msgs::Marker line_strip;

	visualization_msgs::Marker line_strip_path;

	//vector<visualization_msgs::Marker> obs_line_strip;

	visualization_msgs::MarkerArray obs_line_strip;


};

visualize::visualize(ros::NodeHandle& nh)
{
	n = nh;
	marker_initialize = 0;
	obs_marker_initialized = 0;
	path_marker_initialized = 0;
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_pub_obs = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_obs", 1);
	marker_pub_path = n.advertise<visualization_msgs::Marker>("visualization_marker_path", 10);
	draw_polygons_running = 1;
	draw_obstacles_running = 1;
	vis_path_running = 1;
	th_draw_polygons = new boost::thread(&visualize::draw_polygon, this);
	th_draw_obstacles = new boost::thread(&visualize::draw_obstacles, this);
	th_vis_path = new boost::thread(&visualize::vis_path, this);
}

visualize::~visualize()
{
	delete th_draw_polygons;
	delete th_draw_obstacles;
	delete th_vis_path;
	cout<<"threads deleted\n";
}


void visualize::initialize_marker_variables_polygon(bool vertices_done, const environment& env)
{
	line_strip.header.frame_id = "map";
	line_strip.ns = "polygon";
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	line_strip.id = 1;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.scale.x = 0.1;
	line_strip.color.b = 1.0;
	line_strip.color.a = 1.0;
	if (vertices_done)
	{
		geometry_msgs::Point p;
		line_strip.header.stamp = ros::Time::now();
		for (int i = 0; i < env.pboundary.vertices.size(); i++)
		{
			p.x = env.pboundary.vertices[i].x;
			p.y = env.pboundary.vertices[i].y;
			p.z = 0.0;
			line_strip.points.push_back(p);
		}
		if (env.pboundary.vertices.size() > 0)
		{
			p.x = env.pboundary.vertices[0].x;
			p.y = env.pboundary.vertices[0].y;
			line_strip.points.push_back(p);
		}
		cout<<"initialized marker variables\n";
	}	
	marker_initialize = 1;
}

void visualize::initialize_marker_variables_obstacles(int count_obstacles, const environment& env)
{
		// initializing marker variables
	obs_line_strip.markers.resize(env.obstacles.size());
	for (int i = 0; i < env.obstacles.size(); i++)
	{
		geometry_msgs::Point p;
		obs_line_strip.markers[i].header.frame_id = "map";
		stringstream ss;
		ss << i;
		string str = ss.str();
		string ns = "polygon" + str;
		obs_line_strip.markers[i].ns = ns;
		obs_line_strip.markers[i].action = visualization_msgs::Marker::ADD;
		obs_line_strip.markers[i].pose.orientation.w = 1.0;
		obs_line_strip.markers[i].id = i;
		obs_line_strip.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
		obs_line_strip.markers[i].scale.x = 0.1;
		obs_line_strip.markers[i].color.g = 1.0;
		obs_line_strip.markers[i].color.a = 1.0;
		for (int j = 0; j < env.obstacles[i].vertices.size(); j++)
		{
			p.x = env.obstacles[i].vertices[j].x;
			p.y = env.obstacles[i].vertices[j].y;
			p.z = 0.0;
			obs_line_strip.markers[i].points.push_back(p);
		}
		if (env.obstacles[i].vertices.size() > 0)
		{
			p.x = env.obstacles[i].vertices[0].x;
			p.y = env.obstacles[i].vertices[0].y;
			obs_line_strip.markers[i].points.push_back(p);
		}
	}
	cout<<"initialized obstacle marker variables\n";
	obs_marker_initialized = env.obstacles.size() > 0 ? 1 : 0;

}

void visualize::initialize_marker_variables_path(const vector<geometry_msgs::Point>& plan)
{
	geometry_msgs::Point p;
	p.z = 0.0;
	line_strip_path.header.frame_id = "map";
	line_strip_path.ns = "path";
	line_strip_path.action = visualization_msgs::Marker::ADD;
	line_strip_path.pose.orientation.w = 1.0;
	line_strip_path.id = 1;
	line_strip_path.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip_path.scale.x = 0.1;
	line_strip_path.color.r = 1.0;
	line_strip_path.color.a = 1.0;
	line_strip_path.header.stamp = ros::Time::now();
	for (int i = 0; i < plan.size(); i++)
	{
		p.x = plan[i].x;
		p.y = plan[i].y;
		line_strip_path.points.push_back(p);
	}
	cout<<"initialized path marker variables.\n";
	path_marker_initialized = plan.size() > 0 ? 1 : 0;

}

void visualize::draw_polygon()
{
	tf::TransformBroadcaster br;
	tf::Transform trans;
	trans.setOrigin(tf::Vector3(0.0,0.0,0.0));
	tf::Quaternion q;
	q.setRPY(0,0,0);
	trans.setRotation(q);
	while(draw_polygons_running && ros::ok())
	{
		br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "map", "myframe"));
		if (marker_initialize)
		{
			line_strip.header.stamp = ros::Time::now();
			marker_pub.publish(line_strip);
		}
	}
	cout<<"exiting draw polygon thread\n";
}

void visualize::draw_obstacles()
{
	while(draw_obstacles_running && ros::ok())
	{
		if (obs_marker_initialized)
		{
			for (int i = 0; i < obs_line_strip.markers.size(); i++)
			{
				obs_line_strip.markers[i].header.stamp = ros::Time::now();
			}
			marker_pub_obs.publish(obs_line_strip);
		}

	}
	cout<<"exiting draw obstacles thread\n";
}

void visualize::vis_path()
{
	while(vis_path_running && ros::ok())
	{
		ros::Rate r(2);
		r.sleep();
		if (path_marker_initialized)
		{
			line_strip_path.header.stamp = ros::Time::now();
			marker_pub_path.publish(line_strip_path);
		}
	}
	cout<<"exiting visualize path thread\n";
}

#endif
