#include "rrt_star.hpp"

rrtNode::rrtNode(ofVec2f &s, rrtNode *p, double c): p(s), parent(p), children(std::vector<rrtNode * >()), cost(c) {}

rrtNode::rrtNode(const rrtNode &r): p(r.p), parent(r.parent), cost(r.cost) {}

// rrtNode::rrtNode(rrtNode&& other): parent(std::move(other.parent)), p(other.p), cost(other.cost) {}

rrtNode::~rrtNode() {}

rrtStar::rrtStar(): start(nullptr), goal_reached(false), goal_node(nullptr), m(nullptr), dyn_m(nullptr), node_count(0) {
	// assert(m.noCollision(s) and "Start state declared within obstacle.");
	// assert(m.noCollision(e) and "Goal state declared within obstacle.");
	// rrtNode e(s);
	// nodes.push_back(e);
	std::cout << "RRT* initiated!" << std::endl;
}

rrtStar::~rrtStar() {
	std::cout << "Nodes Produced: " << node_count << std::endl;
	freeMemory(start);
	std::cout << "Nodes list cleared!" << std::endl;
}

void rrtStar::freeMemory(rrtNode* node) {
	for (rrtNode *child : node->children) {
		freeMemory(child);
	}
	delete node;
}

void rrtStar::setMaps(Map* map, Map* dyn_map) {
	m = map;
	dyn_m = dyn_map;
	std::cout << "Map has been set!" << std::endl;
}

void rrtStar::setup(const ofVec2f &s, const ofVec2f &e) {
	ofVec2f st(s);
	rrtNode *init = new rrtNode(st);
	start = init;
	goal.set(e);
	// nodes.push_back(init);
	std::cout << "Start  -> (" << start->p.x << ", " << start->p.y << ") Goal -> (" << goal.x << ", " << goal.y << ")" << std::endl;
}

void rrtStar::update() {

	if (node_count < MAX_NODES) {
		// find random point within boundary
		ofVec2f x_rand(ofRandom(0, WIDTH), ofRandom(0, HEIGHT));
		// std::cout << "Random node: (" << x_rand.x << ", " << x_rand.y << ")" << std::endl;
		// find the node nearest
		rrtNode *n_near = nearest(x_rand);
		ofVec2f x_near(n_near->p);
		// std::cout << "Nearest node: (" << x_near.x << ", " << x_near.y << ")" << std::endl;
		// determine point near in the direction of random node at the specified distance
		ofVec2f x_new(steer(x_near, x_rand));
		// std::cout << "New node: (" << x_new.x << ", " << x_new.y << ")" << std::endl;

		if (collisionFree(x_near, x_new)) {
			// n_near becomes parent
			node_count++;
			double cost_new = n_near->cost + hypot(x_near.x - x_new.x, x_near.y - x_new.y);
			rrtNode *n_new = new rrtNode(x_new, n_near, cost_new);
			n_near->children.push_back(n_new);
			// nodes.push_back(n_new);
			// create rrtNode
			// condition for goal reached
			double distance_from_goal = hypot(x_new.x - goal.x, x_new.y - goal.y);
			if (distance_from_goal < GOAL_VICINITY) {
				if (!goal_node) {
					goal_reached = true;
					goal_node = n_new;
				}
				else if (distance_from_goal < hypot(goal_node->p.x - goal.x, goal_node->p.y - goal.y)) {
					goal_node = n_new;
				}
			}
			// std::vector<rrtNode> v(neighborNodes(n_new));
			// std::cout << "Parent for (" << x_new.x << ", " << x_new.y << ") is (" << n_near->p.x << ", " << n_near->p.y << ")" << std::endl;

			// rewiring the nodes
			for (rrtNode *neighbour : neighborNodes(n_new)) {
				rewire(neighbour, n_new);
			}
		}
		else {
			return;
		}
	}


}

void rrtStar::dynamicUpdate() {
	// check for dynamic obstacles
	std::vector<rrtNode*> dangling_nodes{};

	dynamicUpdate_generateDangling(start, dangling_nodes);
	// std::cout << "Working: " << dangling_nodes.size() << std::endl;
	// rejoining the dangling nodes
	for (rrtNode* node : dangling_nodes) {
		rrtNode* parent = nearest(node->p);
		if (collisionFree(node->p, parent->p)) {
			parent->children.push_back(node);
			node->parent = parent;
		}
		else {
			node_count -= dynamicUpdate_updateCount(node);
			// std::cout << "Node count: " << node_count << std::endl;
		}
	}

	// for (rrtNode *node : nodes) {
	// 	// if parent exists and the edge is colliding
	// 	if (node->parent and not dyn_m->noLineCollision(node->p, node->parent->p)) {
	// 		// set the parent of children to NULL
	// 		for (rrtNode *child : node->children) {
	// 			dangling_nodes.push_back(child);
	// 			child->parent = nullptr;
	// 		}
	// 		// delete the node from the children list of parent
	// 		node->parent->children.erase(std::find(node->parent->children.begin(), node->parent->children.end(), node));
	// 		// remove the branch corresponding to the node from nodes list
	// 		removeBranch(node);
	// 		// free the memory
	// 		delete node;
	// 	}
	// }
	// for (rrtNode *node : dangling_nodes) {
	// 	// find nearest node
	// 	rrtNode* new_parent = nearest(node->p);
	// 	std::cout << "Working till here..." << std::endl;
	// 	if (not new_parent) {
	// 		deleteBranch(node);
	// 	}
	// 	else {
	// 		node->parent = new_parent;
	// 		addBranch(node);
	// 	}
	// }
}

void rrtStar::dynamicUpdate_generateDangling(rrtNode *node, std::vector<rrtNode*> &dangling_nodes) {
	// if parent exists and the edge to parent collides
	if (node->parent) {
		if (not dyn_m->noLineCollision(node->p, node->parent->p)) {
			node->parent->children.erase(std::find(node->parent->children.begin(), node->parent->children.end(), node));
			if (dyn_m->noCollision(node->p)) {
				rrtNode* parent = nearest(node->p);
				if (parent) {
					parent->children.push_back(node);
					node->parent = parent;
				}
				else {
					for (rrtNode *child : node->children) {
						// remove parent from child nodes aand
						child->parent = nullptr;
						dynamicUpdate_generateDangling(child, dangling_nodes);
					}
					// delete node
					delete node;
					node_count--;
				}
			}
			else {
				// delete node from children list of parent
				for (rrtNode *child : node->children) {
					// remove parent from child nodes aand
					child->parent = nullptr;
					dynamicUpdate_generateDangling(child, dangling_nodes);
				}
				// delete node
				delete node;
				node_count--;
			}
		}
		else {
			for (rrtNode *child : node->children) {
				dynamicUpdate_generateDangling(child, dangling_nodes);
			}
		}
	}
	else {
		if (node->p == start->p) {
			for (rrtNode *child : node->children) {
				dynamicUpdate_generateDangling(child, dangling_nodes);
			}
		}
		else if (not dyn_m->noCollision(node->p)) {
			for (rrtNode* child : node->children) {
				child->parent = nullptr;
				dynamicUpdate_generateDangling(child, dangling_nodes);
			}
			// delete node
			delete node;
			node_count--;
		}
		else {
			dangling_nodes.push_back(node);
		}
	}

}

unsigned int rrtStar::dynamicUpdate_updateCount(rrtNode *n) {
	if (n->children.empty()) {
		return 1;
	}
	unsigned int deleted_nodes = 1;
	for (rrtNode *child : n->children) {
		deleted_nodes += dynamicUpdate_updateCount(child);
	}
	delete n;
	return deleted_nodes;
}

rrtNode* rrtStar::nearest(ofVec2f &x) const {
	double dist = DBL_MAX;
	return nearestHelper(start, x, dist);
	// double min_dist = DBL_MAX;
	// rrtNode *ans = nullptr;
	// for (rrtNode *node : nodes) {
	// 	// calculate manhattan distance
	// 	double dist = abs(node->p.x - x.x) + abs(node->p.y - x.y);
	// 	if (dist < min_dist) {
	// 		min_dist = dist;
	// 		ans = node;
	// 	}
	// }
	// return ans;
}

rrtNode* rrtStar::nearestHelper(rrtNode *node, const ofVec2f &x, double &dist) const {
	rrtNode *ans = node;
	double node_dist = abs(node->p.x - x.x) + abs(node->p.y - x.y);
	for (rrtNode *child : node->children) {
		rrtNode *new_node = nearestHelper(child, x, dist);
		if (new_node) {
			double child_dist = abs(new_node->p.x - x.x) + abs(new_node->p.y - x.y);
			if (child_dist < node_dist) {
				ans = new_node;
				node_dist = child_dist;
			}
		}
	}
	return node_dist < dist ? ans : nullptr;
}

ofVec2f rrtStar::steer(ofVec2f &x_near, ofVec2f &x_rand) {
	double r = hypot(x_near.x - x_rand.x, x_near.y - x_rand.y);
	double d = RANGE / r;

	if (d >= 1) {
		// if the random point lies within the vicinity of the sensor
		return x_rand;
	}

	double x_new = x_near.x + (x_rand.x - x_near.x) * d;
	double y_new = x_near.y + (x_rand.y - x_near.y) * d;
	return ofVec2f(x_new, y_new);
}

bool rrtStar::collisionFree(const ofVec2f &x_near, const ofVec2f &x_new) const {
	// check if new node is not colliding
	if (m->noCollision(x_new) and dyn_m->noCollision(x_new)) {
		// bool check = ;
		// if (check) {
		// 	std::cout << "Not colliding..." << std::endl;
		// }
		// else {
		// 	std::cout << "Colliding..." << std::endl;
		// }
		return m->noLineCollision(x_near, x_new) and dyn_m->noLineCollision(x_near, x_new);
	}
	return false;
}

std::vector<rrtNode*> rrtStar::neighborNodes(const rrtNode *n, const double d) {

	std::vector<rrtNode*> ans;
	neighborNodesHelper(start, n, ans, d);
	// for (rrtNode *node : nodes) {
	// 	double dist = hypot(node->p.x - n->p.x, node->p.y - n->p.y);
	// 	if (dist < d) {
	// 		ans.push_back(node);
	// 	}
	// }
	return ans;
}

void rrtStar::neighborNodesHelper(rrtNode *node, const rrtNode *x, std::vector<rrtNode*> &ans, const double d) {
	double dist = hypot(node->p.x - x->p.x, node->p.y - x->p.y);
	if (dist < d) {
		ans.push_back(node);
	}

	for (rrtNode* child : node->children) {
		neighborNodesHelper(child, x, ans, d);
	}
}

void rrtStar::rewire(rrtNode *neighbour, rrtNode *new_node) {
	if (collisionFree(neighbour->p, new_node->p)) {
		double dist = new_node->cost + hypot(neighbour->p.x - new_node->p.x, neighbour->p.y - new_node->p.y);
		if (dist < neighbour->cost) {
			neighbour->cost = dist;
			// delete the neighbour node from the children list of its parent
			neighbour->parent->children.erase(std::find(neighbour->parent->children.begin(), neighbour->parent->children.end(), neighbour));
			neighbour->parent = new_node;
			new_node->children.push_back(neighbour);
			// std::cout << "Parent of (" << neighbour->p.x << ", " << neighbour->p.y << ") changed to (" << new_node->p.x << ", " << new_node->p.y << ")" << std::endl;
		}
	}
}
/*
void rrtStar::removeBranch(rrtNode* node) {
	if (node) {
		nodes.erase(std::find(nodes.begin(), nodes.end(), node));
		for (rrtNode *child : node->children) {
			removeBranch(child);
		}
	}
}

void rrtStar::deleteBranch(rrtNode* node) {
	if (node) {
		for (rrtNode *child : node->children) {
			deleteBranch(child);
		}
		delete node;
	}
}

void rrtStar::addBranch(rrtNode* node) {
	nodes.push_back(node);
	for (rrtNode *child : node->children) {
		addBranch(child);
	}
}
*/