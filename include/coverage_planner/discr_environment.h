#ifndef DISCR_ENVIRONENT_H
#define DISCR_ENVIRONMENT_H

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <coverage_planner/environment.h>

using namespace std;

// This code contains data types for storing a discrete representation of the environment 


struct disc_point{               // data type for storing discrete points
	int x, y;
	disc_point(int, int);
	bool operator == (const disc_point& d) const;
	bool operator != (const disc_point& d) const;
};

disc_point::disc_point(int x_ = 0, int y_ = 0)
{
	x = x_;
	y = y_;
}

bool disc_point::operator == (const disc_point& d) const
{
	return (x == d.x && y == d.y);
}

bool disc_point::operator != (const disc_point& d) const
{
	return !(x == d.x && y == d.y);
}

struct node{            // data type for storing node attributes, i.e. discrete point, parent, child and flag
	disc_point p;
	disc_point parent, child;
	vector<disc_point> neighbors;
	disc_point qparent;
	bool inqueue; 
	bool nodereached;
	node();
};

node::node()
{
	parent.x = -1;
	parent.y = -1;
	nodereached = false;
}

struct classcomp {
  bool operator() (const disc_point& lhs, const disc_point& rhs) const
  {	
  	if (lhs.x < rhs.x)
  		{ return 1;}
  	else if (lhs.x > rhs.x)
  		{	return 0; }
  	else if (lhs.x == rhs.x)
  	{
  		return lhs.y < rhs.y;
  	}
  }
};


class discr_environment     // this class stores a set and a map for containing discrete point values and nodes of the environment
{
public:
	
	set<disc_point,classcomp> pointset;

	map<disc_point,node,classcomp> graphnodes;

	int x_cells, y_cells;

	void initialize_discr_environment(const environment&);

};

void discr_environment::initialize_discr_environment(const environment& e)
{
	x_cells = floor((abs(e.xmax - e.xmin))/e.resolution);
	y_cells = floor((abs(e.ymax - e.ymin))/e.resolution);
}

#endif