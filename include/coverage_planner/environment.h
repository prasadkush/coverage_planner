#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

// This code contains data type and functions for representing the environment in form of polygons and obstacle polygons

struct vertex     // data type for storing x, y values of a vertex and also for storing continuous x, y point values
{
	double x, y;
	vertex(double x_, double y_);
};

vertex::vertex(double x_ = 0.0, double y_ = 0.0)
{
	x = x_;
	y = y_;
}

struct polygon     // polygon represented as a vector of x, y values
{
	vector<vertex> vertices;
};

struct environment
{
	polygon pboundary;   // pboundary stores the input polygon 
	vector<polygon> obstacles;   // obstacles store the input polygonal obstacles
	double xmin, ymin, xmax, ymax;   
	double resolution;       
	environment();
	void compute_env_bounds();
	void compute_resolution();
	void output_parameters();
	void initialize_environment();
};

environment::environment()
{
	resolution = 0.05;
	xmin = 0.0; 
	ymin = 0.0;
	xmax = 1.0;
	ymax = 1.0;
}

void environment::compute_env_bounds()   // computes min and mx x,y, values from input polygon
{
	xmin = pboundary.vertices[0].x;
	ymin = pboundary.vertices[0].y;
	xmax = pboundary.vertices[0].x;
	ymax = pboundary.vertices[0].y;
	for (int i = 0; i < pboundary.vertices.size(); i++)
	{
		xmin = pboundary.vertices[i].x < xmin ? pboundary.vertices[i].x : xmin;
		ymin = pboundary.vertices[i].y < ymin ? pboundary.vertices[i].y : ymin;
		xmax = pboundary.vertices[i].x > xmax ? pboundary.vertices[i].x : xmax;
		ymax = pboundary.vertices[i].y > ymax ? pboundary.vertices[i].y : ymax;
	}
}

void environment::compute_resolution()
{
	resolution = (min(abs(xmin - xmax), abs(ymin - ymax)))/10;
	resolution = 0.25;
}

void environment::output_parameters()
{
	cout<<"xmin: "<<xmin<<" xmax: "<<xmax<<"\n";
	cout<<"ymin: "<<ymin<<" ymax: "<<ymax<<"\n";
	cout<<"resolution: "<<resolution<<"\n";
}

void environment::initialize_environment()
{
	compute_env_bounds();
	compute_resolution();
	output_parameters();
}

#endif