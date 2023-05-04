#pragma once

#include "ofMain.h"
#include <math.h>
#include <limits.h>
#include "defParams.hpp"
#include "map.hpp"

class rrtNode {
public:
	rrtNode(ofVec2f&, rrtNode *p = nullptr, double c = 0);
	rrtNode(const rrtNode&); // copy constructor for node
	rrtNode& operator=(const rrtNode&) = delete;
	// rrtNode(rrtNode&&) = default;
	// rrtNode& operator=(rrtNode&&) = delete;
	~rrtNode();

	ofVec2f p; // the start and end point of the edge
	rrtNode* parent; // pointer to parent edge
	std::vector<rrtNode*> children;
	double cost; // cost till the `end` point
};

class rrtStar {
public:
	rrtStar();

	void setMaps(Map* map, Map* = nullptr);
	/*
	Sets the map
	*/

	void setup(const ofVec2f&, const ofVec2f&);
	/*
	Set the start and end goal respectively
	*/

	void update();
	/*
	Makes the graph for rrt
	*/

	void dynamicUpdate();
	/*
	Updates the dynamic map
	*/

	// std::vector<rrtNode*> nodes;
	rrtNode* start;
	ofVec2f goal;
	bool goal_reached;
	rrtNode *goal_node;

	~rrtStar();

private:
	rrtNode* nearest(ofVec2f&) const;
	/*
	Returns the nearest node present in the
	*/

	rrtNode* nearestHelper(rrtNode*, const ofVec2f&, double&) const;
	/*
	A helper funciton for nearest function
	*/

	ofVec2f steer(ofVec2f&, ofVec2f&);

	bool collisionFree(const ofVec2f &x_near, const ofVec2f &x_new) const;
	/*
	Checks if the line segment made by x_near and x_new is collision free or not
	*/

	std::vector<rrtNode*> neighborNodes(const rrtNode*, const double d = NEIGHBOUR_RANGE);
	/*
	Returns a vector containing neighbouring nodes in a certain radius
	*/

	void neighborNodesHelper(rrtNode*, const rrtNode*, std::vector<rrtNode*>&, const double);
	/*
	A helper function for neighbourNodes
	*/

	void rewire(rrtNode*, rrtNode*);
	/*
	Rewires the rrt map
	*/

	void dynamicUpdate_generateDangling(rrtNode*, std::vector<rrtNode*>&);
	/*
	A helper function for dynamicUpdate function
	*/

	unsigned int dynamicUpdate_updateCount(rrtNode*);
	/*
	Deletes the corresponding branch and free the memory
	*/

	void freeMemory(rrtNode*);
	/*
	Free the memory occupied by rrt graph
	*/

	Map *m;
	Map *dyn_m;
	unsigned int node_count;
};