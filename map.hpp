#ifndef MAP_HPP
#define MAP_HPP

#include <ofMain.h>
#include "defParams.hpp"
#include <vector>
#include <iostream>
#include <cassert>
#include <stdexcept>

/*
	screen 1600x900
*/

// ********* EDGE *************
class Edge {
	// the left side of the edge represent obstacle
public:
	Edge(ofVec2f&, ofVec2f&);

	bool onLeft(const ofVec2f&) const;
	/*
	Checks if the object lies on the left of the edge or not.
	Returns true if point lies on left
	*/

	bool is_intersecting(const ofVec2f&, const ofVec2f&) const;
	/*
	calculates whether there is intersection of line segment defined by two points with the edge
	*/

	friend class Obstacle;

	ofVec2f start;
	ofVec2f end;

private:
	bool is_between(const ofVec2f&, const ofVec2f&, double x, double y) const;
	/*
	Checks if the point (x, y) lies between the points given
	*/
};

// ********* OBSTACLE *********
class Obstacle {
public:
	virtual bool noCollision(const ofVec2f&) const = 0;
	/*
	Consider only concave shapes
	*/

	virtual bool noLineCollision(const ofVec2f&, const ofVec2f&) const = 0;
	/*
	Checks if the line segment defined by the two points intersects the polygon
	*/

	virtual ~Obstacle();
	std::string obstacle_type;

protected:
	Obstacle(const std::string&);
	/*
	Checks if the new point collides with the obstacle
	*/

};

// ********* Polygon -> Obstacle *********
class Polygon: public Obstacle {
public:
	Polygon(std::vector<ofVec2f> &v, const std::string &name = "");
	/*
	Makes a closed polygon from the fiven vertices
	*/

	~Polygon();
	std::vector<Edge> edges; // edge list

protected:

	std::vector<ofVec2f>& shoelace(std::vector<ofVec2f>&);

	void addFirstEdge(ofVec2f&, ofVec2f&);
	/*
	Adds the first edge by given two vertices
	*/

	void addEdge(ofVec2f&);
	/*
	Adds new edge to the obstacle polygon by passing new vertex
	*/

	void closePolygon();
	/*
	Joins the last vertex to the first vertex, closing the polygon
	*/

	bool noCollision(const ofVec2f&) const;
	/*
	Checks collision for polygon.
	Returns true if no collision
	*/

	bool noLineCollision(const ofVec2f&, const ofVec2f&) const;
	/*
	Checks collision of line segment with the polygon
	*/

	void polyErr();
	/*
	Checks if the defined obstacle is of valid data type or not.
	*/

	void checkPolygon();
	/*
	Checks if the given polygon is valid or not
	*/

	uint8_t sides;

private:
	void makeAddEdge(ofVec2f&, ofVec2f&);
	/*
	Checks if the polygon is valid
	Makes an edge given two vertices
	*/

	bool closed; // determines if the polygon is closed or not
};

// ********* Conic -> Obstacle *********
class Conic: public Obstacle {
public:
	ofVec2f center; // center if the conic
	double a, b; // major and minor axis respectively

protected:
	Conic(ofVec2f, double, double, const std::string&);
	/*
	Initialize the conic given the center coordinates as well as major and minor axis
	((center_x, center_y), major_axis_a, minor_axis_b)
	*/

	bool noCollision(const ofVec2f&) const;
	/*
	Check for collision of a point with the conic.
	Returns true if no collision
	*/

	bool noLineCollision(const ofVec2f&, const ofVec2f&) const;
	/*
	Checks collision of line segment with the conic
	*/

	bool is_between(const ofVec2f&, const ofVec2f&, double x, double y) const;
	/*
	Checks if the point (x, y) lies between the points given
	*/

	~Conic();
};

// ********* Triangle -> Polygon *********
class Triangle: public Polygon {
public:
	Triangle(const std::vector<ofVec2f> &v = std::vector<ofVec2f>());
	~Triangle();
};

// ********* Quadrilateral -> Polygon *********
class Quadrilateral: public Polygon {
public:
	Quadrilateral(const std::vector<ofVec2f> &v = std::vector<ofVec2f>(), const std::string &name = "Quadrilateral");
	~Quadrilateral();
};

// ********* Ellipse -> Conic*********
class Ellipse: public Conic {
public:
	Ellipse(ofVec2f&, double, double, const string &name = "Ellipse");
	~Ellipse();
};

// ********* Circle -> Conic*********
class Circle: public Ellipse {
public:
	Circle(ofVec2f&, double);
	~Circle();
};

// ********* Rectangle -> Quadrilateral*********
class Rectangle: public Quadrilateral {
public:
	Rectangle(ofVec2f&, double, double, const std::string &name = "Rectangle");
	virtual ~Rectangle();
};

// ********* Square -> Rectangle*********
class Square: public Rectangle {
public:
	Square(ofVec2f&, double);
	~Square();
};

// ********* Map *********
class Map {
public:
	Map(uint16_t w = WIDTH, uint16_t h = HEIGHT);
	/*
	If one of the parameter is not provided, default values are used
	*/

	void setup(std::vector<Obstacle *>&);
	/*
	Sets a map from the list of obstacles, width and height given.
	*/

	void addObstacle(Obstacle*);
	/*
	Adds an obstacle to the map
	*/

	bool noCollision(const ofVec2f&) const;
	/*
	Checks if the given vertex collides with any obstacle or not
	*/

	bool noLineCollision(const ofVec2f&, const ofVec2f&) const;
	/*
	Checks if there is no collision of the line segment with the obstacles
	*/

	std::vector<Obstacle *> obstacles;

	~Map();

private:
	uint16_t width, height;
};
#endif