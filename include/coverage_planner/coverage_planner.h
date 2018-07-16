#ifndef COVERAGE_PLANNER_
#define COVERAGE_PLANNER_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <coverage_planner/environment.h>
#include <coverage_planner/discr_environment.h>
#include <coverage_planner/visualize.h>

struct cont_point{
	double x, y;
};

class coverage_planner
{
public:

	ros::NodeHandle n;

	ros::Subscriber sub_polygon;

	//ros::Publisher marker_pub, marker_pub_obs;

	environment env;

	discr_environment envd;

	coverage_planner(ros::NodeHandle& nh);

	~coverage_planner();

	visualize vis;

	bool exit_condition;

	static void exit_gracefully(coverage_planner* c);

	void compute_plan();   // function which does all the plan computation

	boost::thread* th_input;

	//boost::thread th_exit;

private:

	void vertices_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);

	void input_polygon();	// input thread function

	void define_motions();

	void continuous_to_discrete(double x, double y, int& x_dis, int& y_dis, const environment& e);

	void discrete_to_continuous(int x, int y, double& cx, double& cy, const environment& e);

	bool valid_polygon(const polygon& p);

	void compute_line_eqn(double& a, double& b, double& c, const vertex& v1, const vertex& v2);

	bool line_segment_intersect(const vertex& v1, const vertex& v2, const vertex& v3, const vertex& v4);

	bool point_within_polygon(double x, double y, const polygon& p);

	void iterate_over_adjacent_nodes(bool& edge_added, disc_point& adj, disc_point& current);

	void iterate_over_adjacent_nodes_(bool& edge_added, disc_point& adj, disc_point& current);

	bool within_bounds(disc_point p, int x_cells, int y_cells);

	bool within_obstacle(double, double, const environment& e);

	disc_point compute_adjacent(const disc_point& p, string m);

	void compute_source(vertex& v);

	void make_graph(environment& e, discr_environment& d);

	bool add_node(disc_point adj, disc_point current, const environment& e, discr_environment& d);

	void shortest_path(disc_point s, disc_point g, vector<geometry_msgs::Point>& plan_, discr_environment& d);

	void output_plan(const vector<geometry_msgs::Point>&);

	int count_vertices, count_obstacles, no_obstacles;

	vertex test_vertex;

	vector<string> motions;

	vector<geometry_msgs::Point> plan;

	bool vertices_done, obstacles_done;

};

#endif
