// Code written by Kush Prasad //
// email: prasad.kush@gmail.com //

#include <ros/ros.h>
#include <iostream>
#include <queue>
#include <coverage_planner/coverage_planner.h>
#include <tf/transform_broadcaster.h>

/*
This code computes a coverage plan for a convex polygon environment with obstacles (also as convex polygons)
The environment is taken as input from rviz and stored in environment variables
The input environment is checked wether it is valid (convex polygon). If not, the program exits.
The above is done in input_polygon() function which is called as a thread from the constructor.
For a valid environment, environment variables (resolution, bounds) are initialized
The coverage plan is then computed by a call to the compute_plan() function from the input_polygon() function

Assumptions:

1) The environment is bounded by a convex polygon and contains obstacles as convex polygons
2) The robot only executes the motions up, left, down and right.

*/

coverage_planner::coverage_planner(ros::NodeHandle& nh) : vis(nh)
{

	n = nh;

	count_vertices = 0;
	vertices_done = 0;
	count_obstacles = 0;
	obstacles_done = 0;

	exit_condition = false;

	define_motions();    

	sub_polygon = n.subscribe("/initialpose", 1, &coverage_planner::vertices_callback, this);
	
	th_input = new boost::thread(&coverage_planner::input_polygon, this);

}

coverage_planner::~coverage_planner()
{
	exit_condition = true;
	delete th_input;
}


void coverage_planner::define_motions()
{
	motions.resize(4);
	motions[0] = "down";  
	motions[1] = "right";
	motions[2] = "up";
	motions[3] = "left";
}

void coverage_planner::exit_gracefully(coverage_planner* c)
{
	ros::Rate r(2);
	//while(ros::ok())
	//{
	cout<<"exit_condition: "<<c->exit_condition<<"\n";
	if (c->exit_condition)
	{
		cout<<"exiting due to invalid input\n";
		c->vis.draw_polygons_running = false;
		c->vis.draw_obstacles_running = false;
		c->vis.vis_path_running = false;
		c->vis.marker_initialize = false;
		c->vis.obs_marker_initialized = false;
		c->vis.path_marker_initialized = false;
		c->vis.th_draw_polygons->join();
		c->vis.th_draw_obstacles->join();
		c->vis.th_vis_path->join();
	//c->th_input->join();
		cout<<"all threads done\n";
		ros::shutdown();
		exit(0);
	}
	//r.sleep();
	//}	
}

// this function is the subscriber callback function for input values from rviz
// It stores the values in environment variables

void coverage_planner::vertices_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	cout<<"received point\n";
	if (vertices_done == 0)
	{
		vertex input_vert(msg.pose.pose.position.x, msg.pose.pose.position.y);
		env.pboundary.vertices.push_back(input_vert);
		count_vertices += 1;
	}
	if (vertices_done == 1 && no_obstacles > 0)
	{
		vertex input_vert(msg.pose.pose.position.x, msg.pose.pose.position.y);
		env.obstacles[count_obstacles].vertices.push_back(input_vert);
	}
	if (vertices_done == 1 && obstacles_done == 1)
	{
		vertex input_vert(msg.pose.pose.position.x, msg.pose.pose.position.y);
		test_vertex = input_vert;
		cout<<"point x: "<<test_vertex.x<<" y: "<<test_vertex.y<<"\n";
	}
}

// this function checks wether the input polygons are valid (convex) or not. it exits the program if not.
// After taking inputs and checking valid polygons, it initializes environment variables (resolution) ...
// ... and then calls the compute_plan fuction which processes the coverage plan algorithm 

void coverage_planner::input_polygon()
{
	ros::Rate r(30);
	int n;
	cout<<"\nPlease enter the polygon as vertice points on rviz using 2D Pose Estimate button.\n";
	cout<<"The orientation does not matter. After done, please enter a digit.\n";
	cin>>n;
	vertices_done = 1;
	no_obstacles = 0;
	r.sleep();
	bool polygonvalid = valid_polygon(env.pboundary);
	cout<<"polygon valid: "<<polygonvalid<<"\n";
	vis.initialize_marker_variables_polygon(vertices_done, env);
	if (polygonvalid == false)
	{
		exit_condition = true;
	}
	if (polygonvalid == true)
	{
		cout<<"\nPlease enter number of obstacles to be entered\n";
		cin>>no_obstacles;
		cout<<"no of obstacles: "<<no_obstacles<<"\n";
		if (no_obstacles < 0)
		{
			exit_condition = true;
		}
	}

	if (no_obstacles > 0)
	{ 
		env.obstacles.resize(max(0,no_obstacles));
		for (int i = 0; i < no_obstacles; i++)
		{
			cout<<"\nPlease click polygon obstacle points on rviz using 2D Pose Estimate button and then enter a digit.\n";
			cin>>n;
			count_obstacles += 1;
		}
		cout<<"Note: only convex and polygon obstacles will be considered.\n";
		r.sleep();
		vector<polygon>::iterator it;
		it = env.obstacles.begin();
		while (it != env.obstacles.end())   // this loop erases invalid (non polygon and non-convex) obstacles
		{
			if (!valid_polygon(*it))
			{
				it = env.obstacles.erase(it);
			}
			else
			{
				it++;
			}
		}
		cout<<"no of valid obstacles: "<<env.obstacles.size()<<"\n";
		vis.initialize_marker_variables_obstacles(count_obstacles, env);
	}
	obstacles_done = 1;

	if (!exit_condition)
	{	
		env.initialize_environment();
		compute_plan();	 	
	}
}


void coverage_planner::continuous_to_discrete(double x, double y, int& x_dis, int& y_dis, const environment& e)
{
	x_dis = floor(max((min(x,e.xmax) - e.xmin),0.0)/e.resolution);
	y_dis = floor(max((min(y,e.ymax) - e.ymin),0.0)/e.resolution);
}

void coverage_planner::discrete_to_continuous(int x, int y, double& cx, double& cy, const environment& e)
{
	cx = (x + 0.5)*e.resolution + e.xmin;
	cy = (y + 0.5)*e.resolution + e.ymin;
}

//this function determines if given vertices form a convex polygon

bool coverage_planner::valid_polygon(const polygon& p)
{
	double a, b, c, val;
	int psize = p.vertices.size(), valsign, prevvalsign, count;
	//cout<<"psize: "<<psize<<"\n";
	for (int i = 0; i < psize; i++)
	{
		int k = (i + 1) % psize;
		compute_line_eqn(a,b,c,p.vertices[i],p.vertices[k]);
		if (a != 0 || b != 0)
		{
			val = 1.0; valsign = 1; prevvalsign = 1;
			count = 0;
			for (int j = 0; j < p.vertices.size(); j++)
			{
				if (j != i && j != k)
				{
					prevvalsign = valsign;
					val = (a*p.vertices[j].x + b*p.vertices[j].y + c)/sqrt(a*a + b*b);
					valsign = val >= 0 ? 1 : -1;
					count++;
				}
				if (count > 1 && valsign*prevvalsign < 0)
				{ return false; }
			}
		}
	}
	if (psize <= 2)
	{ return false; }

	return true;
}

//this function determines if a point lies within a polygon

bool coverage_planner::point_within_polygon(double x, double y, const polygon& p)
{
	double a, b, c, val, pointval;
	int psize = p.vertices.size(), valsign, pointvalsign, count;
	for (int i = 0; i < psize; i++)
	{
		int k = (i + 1) % psize;
		compute_line_eqn(a,b,c,p.vertices[i],p.vertices[k]);
		pointval = (a*x + b*y + c)/sqrt(a*a + b*b);
		if (a != 0 || b != 0)
		{
			count = 0;
			for (int j = 0; j < p.vertices.size(); j++)
			{
				valsign = 1; pointvalsign = 1;
				if (j != i && j != k)
				{
					val = (a*p.vertices[j].x + b*p.vertices[j].y + c)/sqrt(a*a + b*b);					
					valsign = val >= 0 ? 1 : -1;
					pointvalsign = pointval >= 0 ? 1 : -1;
					count++;
				}
				if (valsign * pointvalsign < 0)
				{  
					return false; 
				}
			}
		}
	}
	if (psize <= 2)
	{ return false; }

	return true;
}


//this function determines if two line segments intersect

bool coverage_planner::line_segment_intersect(const vertex& v1, const vertex& v2, const vertex& v3, const vertex& v4)
{
	double a1, b1, c1, a2, b2, c2, m1, m2, x_intersect, y_intersect;
	compute_line_eqn(a1,b1,c1,v1,v2);
	compute_line_eqn(a2,b2,c2,v3,v4);
	if ((a2*b1 - a1*b2) == 0.0)
	{
		return false;
	}	
	else 
	{
		x_intersect = (b2*c1 - b1*c2)/(a2*b1 - b2*a1);
		y_intersect = (a1*c2 - a2*c1)/(a2*b1 - b2*a1);
		double alpha1, alpha2;
		alpha1 = b1 != 0.0 ? (x_intersect - v1.x)/(v2.x - v1.x) : (y_intersect - v1.y)/(v2.y - v1.y);
		alpha2 = b2 != 0.0 ? (x_intersect - v3.x)/(v4.x - v3.x) : (y_intersect - v3.y)/(v4.y - v3.y);
		if (alpha1 >= 0 && alpha1 <= 1 && alpha2 >= 0 && alpha2 <= 1)
		{
			return true;
		}
		else 
		{
			return false;
		}
	}

}


// this function computes equation of a line given two points and stores the eqn ax + by + c = 0 in a,b,c.

void coverage_planner::compute_line_eqn(double& a, double& b, double& c, const vertex& v1, const vertex& v2)
{
	if (v1.x == v2.x && v1.y == v2.y)
	{
		a = 0.0; b = 0.0; c = 0.0;
	}
	if (v1.x == v2.x && v1.y != v2.y)
	{
		a = 1.0;
		b = 0.0;
		c = -v1.x;
	}
	else if (v1.x != v2.x && v1.y == v2.y)
	{
		a = 0.0;
		b = 1.0;
		c = -v1.y;
	}
	else 
	{
		double slope;
		slope = (v2.y - v1.y)/(v2.x - v1.x);
		a = -slope;
		b = 1.0;
		c = -(a*v2.x + b*v2.y);
	}
}

//this function determines wether a given point is within any obstace in the environment

bool coverage_planner::within_obstacle(double x, double y, const environment& e)
{
	if (!point_within_polygon(x,y,e.pboundary))
	{
		return true;
	}
	for (int i = 0; i < e.obstacles.size(); i++)
	{
		if(point_within_polygon(x,y,e.obstacles[i]))
		{
			return true; 
		}
	}
	return false;
}


//this function stores discrete nodes in a set and in a map

void coverage_planner::make_graph(environment& e, discr_environment& d)
{
	e.initialize_environment();
	d.x_cells = floor((abs(e.xmax - e.xmin))/e.resolution);
	d.y_cells = floor((abs(e.ymax - e.ymin))/e.resolution);
	cout<<"x_cells: "<<d.x_cells<<" y_cells: "<<d.y_cells<<"\n";
	double x, y;
	for (int i = 0; i <= d.x_cells; i++)
	{
		for (int j = 0; j <= d.y_cells; j++)
		{
			discrete_to_continuous(i,j,x,y,e);
			if (!within_obstacle(x,y,e))
			{
				node n;
				vector<disc_point> neighbors;
				disc_point p(i,j), adj;
				n.p = p;
				n.nodereached = false;
				for (int k = 0; k < motions.size(); k++)
				{
					adj = compute_adjacent(p, motions[k]);
					discrete_to_continuous(adj.x,adj.y,x,y,e);
					if (within_bounds(adj,d.x_cells,d.y_cells) && !within_obstacle(x,y,e))
					{
						neighbors.push_back(adj);
					}
				}
				n.neighbors = neighbors;
				d.pointset.insert(p);
				d.graphnodes[p] = n;
			}			
		}
	}
}


void coverage_planner::compute_source(vertex& v)
{
	double xmin = env.pboundary.vertices[0].x;
	v = env.pboundary.vertices[0];
	for (int i = 0; i < env.pboundary.vertices.size(); i++)
	{
		if (env.pboundary.vertices[i].x < xmin)
		{
			xmin = env.pboundary.vertices[i].x;
			v = env.pboundary.vertices[i];
		}
	}
}

//compute adjacent node given current node and a motion

disc_point coverage_planner::compute_adjacent(const disc_point& p, string m)
{
	if (m == "down")
	{
		disc_point adj(p.x,p.y-1);
		return adj;
	}
	else if (m == "right")
	{
		disc_point adj(p.x+1,p.y);
		return adj;
	}
	else if (m == "up")
	{
		disc_point adj(p.x,p.y+1);
		return adj;
	}
	else if (m == "left")
	{
		disc_point adj(p.x-1,p.y);
		return adj;
	}
	else
	{
		disc_point adj;
		adj.x = p.x; adj.y = p.y;
		return adj;
	}
}

bool coverage_planner::within_bounds(disc_point p, int x_cells, int y_cells)
{
	return (p.x >= 0 && p.x <= x_cells) && (p.y >= 0 && p.y <= y_cells);
}


void coverage_planner::shortest_path(disc_point s, disc_point g, vector<geometry_msgs::Point>& plan_, discr_environment& d)
{
	map<disc_point,node>::iterator it;
	queue<disc_point> q;
	disc_point current;
	double x, y;
	geometry_msgs::Point p;
	p.z = 0.0;
	for (it = d.graphnodes.begin(); it != d.graphnodes.end(); it++)
	{
		it->second.inqueue = false;
	}
	q.push(s);
	d.graphnodes[s].inqueue = true;
	current = s;
	while (!q.empty() && current != g)
	{
		vector<disc_point> neigh;
		neigh = d.graphnodes[current].neighbors;
		for (int i = 0; i < neigh.size(); i++)
		{
			if (!d.graphnodes[neigh[i]].inqueue)
			{	q.push(neigh[i]);
				d.graphnodes[neigh[i]].inqueue = true;
				d.graphnodes[neigh[i]].qparent = current;
			}
		}
		q.pop();
		current.x = q.front().x;
		current.y = q.front().y;
	}
	vector<geometry_msgs::Point> temp_plan;
	if (current == g && s != g)
	{
		while (current != s)
		{
			discrete_to_continuous(current.x,current.y,x,y,env);
			p.x = x;
			p.y = y;
			temp_plan.push_back(p);
			current = d.graphnodes[current].qparent;
		}
		if (temp_plan.size() > 0)
		{
			for (int i = temp_plan.size() - 1; i >= 0; i--)
			{	
				plan_.push_back(temp_plan[i]);
			}
		}	
	}

}

bool coverage_planner::add_node(disc_point adj, disc_point current, const environment& e, discr_environment& d)
{
	double x,y;
	discrete_to_continuous(adj.x,adj.y,x,y,e);
	if (within_bounds(adj,d.x_cells,d.y_cells) && !within_obstacle(x,y,e) && d.graphnodes[adj].nodereached == 0)
	{
		d.graphnodes[adj].parent = current;
		d.graphnodes[current].child = adj;
		d.graphnodes[adj].nodereached = true;
		return true;
	}
	return false;
}

void coverage_planner::iterate_over_adjacent_nodes_(bool& edge_added, disc_point& adj, disc_point& current)
{
	double x, y;
	for (int i = 0; i < motions.size(); i++)
	{
		if (!edge_added)
		{
			adj = compute_adjacent(current,motions[i]);
			discrete_to_continuous(adj.x,adj.y,x,y,env);
			if (within_bounds(adj,envd.x_cells,envd.y_cells) && !within_obstacle(x,y,env) && envd.graphnodes[adj].nodereached == 0)
			{ edge_added = true; }
		}
		if (edge_added)
		{
			break;
		}
	}
}

void coverage_planner::iterate_over_adjacent_nodes(bool& edge_added, disc_point& adj, disc_point& current)
{
	for (int i = 0; i < motions.size(); i++)
	{
		if (!edge_added)
		{
			adj = compute_adjacent(current,motions[i]);
			edge_added = add_node(adj,current,env,envd);
		}
		if (edge_added)
		{
			current = adj;
			break;
		}
	}
}

void coverage_planner::compute_plan()
{
	vertex v;
	disc_point source, current, adj;
	geometry_msgs::Point p;
	int no_of_jumps = 0;
	p.z = 0.0;
	double x, y;
	set<disc_point>::iterator it;
	make_graph(env,envd);        // fill discrete valid node values in a set and a map
	it = envd.pointset.begin();
	current.x = it->x;
	current.y = it->y;
	source = current;
	envd.graphnodes[current].nodereached = true;
	discrete_to_continuous(current.x,current.y,p.x,p.y,env);
	plan.push_back(p);
	while (!envd.pointset.empty())
	{
		bool edge_added = false;
		it = envd.pointset.find(current);   //remove node being expanded from set
		if (it != envd.pointset.end())
		{ envd.pointset.erase(it); }
        iterate_over_adjacent_nodes(edge_added,adj,current);  // consider adjacent nodes for motion in counter-clock wise direction
		if (edge_added)										  // only obstacle free and nodes not yet reached are considered valid
		{
			discrete_to_continuous(adj.x,adj.y,p.x,p.y,env);
			plan.push_back(p);    // add to plan if a valid adjacent node is found
			continue;
		}
		else 
		{
			if (current == source)
			{
				break;
			}
			else
			{
				disc_point start;
				start = current;
				while(!edge_added && current != source)     // this loop entered when no obstacle free adjacent node is ...
				{											// ... present. we then go back to a node in the path from which ...
					current = envd.graphnodes[current].parent;   // ... a valid adjacent node is reachable 
					iterate_over_adjacent_nodes_(edge_added,adj,current);
					if (edge_added)
					{
						shortest_path(start,current,plan,envd);
						discrete_to_continuous(current.x,current.y,p.x,p.y,env);
						plan.push_back(p);
						no_of_jumps += 1;
					}
				}
			}
		}

	}
	vis.initialize_marker_variables_path(plan);
	output_plan(plan);
	cout<<"\nsolution found\n";
	cout<<"plan size: "<<plan.size()<<"\n";
	cout<<"jumps: "<<no_of_jumps<<"\n";
	//cout<<"used shortest_path\n";

}

void coverage_planner::output_plan(const vector<geometry_msgs::Point>& plan)
{
	for (int i = 0; i < plan.size(); i++)
	{
		cout<<i+1<<". x: "<<plan[i].x<<"  y: "<<plan[i].y<<"\n";
	}
}





/*
int main(int argc, char** argv)
{
	ros::init(argc, argv, "planner");
	tf::TransformListener tf_(ros::Duration(10));
	coverage_planner planner;
	ros::spin();
	return 0;
}
*/