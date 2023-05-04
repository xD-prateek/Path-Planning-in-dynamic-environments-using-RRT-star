#include "map.hpp"

// ********* EDGE *************
Edge::Edge(ofVec2f &s, ofVec2f &e): start(s), end(e) {
	std::cout << "Created edge from (" << s.x << ", " << s.y << ") to (" << e.x << ", " << e.y << ")." << std::endl;
}

bool Edge::onLeft(const ofVec2f &v) const {
	return v.y * (end.x - start.x) - v.x * (end.y - start.y) + start.x * end.y - end.x * start.y <= 0;
}

bool Edge::is_intersecting(const ofVec2f &s, const ofVec2f &e) const {

	double denom = (end.x - start.x) * (e.y - s.y) - (e.x - s.x) * (end.y - start.y);
	if (denom) {
		double t1 = end.x * start.y - start.x * end.y, t2 = e.x * s.y - s.x * e.y;
		double x = ((e.x - s.x) * t1 - (end.x - start.x) * t2) / denom;
		double y = ((e.y - s.y) * t1 - (end.y - start.y) * t2) / denom;
		// bool ans = is_between(s, e, x, y) and is_between(start, end, x, y);
		// if (ans) {
		// 	std::cout << "is intersecting" << std::endl;
		// }
		// else {
		// 	std::cout << "is not intersecting" << std::endl;
		// }
		// std::cout << "intersection point (" << x << ", " << y << ") between lines defined by l1: (" << start.x << ", " << start.y << ") (" << end.x << ", " << end.y << ") and l2: (" << s.x << ", " << s.y << ") (" << e.x  << ", " << e.y << ")" << std::endl;
		// return ans;
		return is_between(s, e, x, y) and is_between(start, end, x, y);
	}

	return is_between(start, end, s.x, s.y) or is_between(start, end, e.x, e.y);
}

bool Edge::is_between(const ofVec2f& s, const ofVec2f& e, double x, double y) const {
	// round thevalues upto 2 decimal places
	x = std::round(x * 100) / 100.0;
	y = std::round(y * 100) / 100.0;
	// bool ans = x <= std::max(s.x, e.x) and x >= std::min(s.x, e.x) and y <= std::max(s.y, e.y) and y >= std::min(s.y, e.y);
	// if (ans) {
	// 	std::cout << "is between!" << std::endl;
	// }
	// else {
	// 	std::cout << "is not between!" << std::endl;
	// }
	// return ans;
	return x <= max(s.x, e.x) and x >= min(s.x, e.x) and y <= max(s.y, e.y) and y >= min(s.y, e.y);
}

// ********* OBSTACLE *********
Obstacle::Obstacle(const string &name): obstacle_type(name) {
	std::cout << "Initiated obstacle creation..." << std::endl;
}

Obstacle::~Obstacle() {
	std::cout << "Obstacle removed." << std::endl;
}

// ********* Polygon *********
Polygon::Polygon(std::vector<ofVec2f> &v, const string &name): Obstacle(name), sides(v.size()), closed(false) {
	// Obstacle called implicitly
	if (!sides) { // if an empty vector is passed
		std::cout << "Empty polygon initiated..." << std::endl;
		return;
	}


	assert(n >= 2 and "One vertex cannot define polygon!");
	addFirstEdge(v[0], v[1]);

	for (int i = 2; i <= sides; ++i) {
		addEdge(v[i]);
	}

	std::cout << "Polygon: " << std::endl;
}

Polygon::~Polygon() {
	std::cout << "Polygon removed." << std::endl;
}

std::vector<ofVec2f>& Polygon::shoelace(std::vector<ofVec2f> &x) {
	int n = x.size();
	long area = 0;

	for (int i = 0; i < n; ++i) {
		int j = (i + 1) % n;
		area += x[i].x * x[j].y - x[i].y * x[j].x;
	}

	if (area > 0) {
		std::reverse(x.begin(), x.end());
	}

	for (ofVec2f o : x) {
		std::cout << "(" << o.x << ", " << o.y << ") ";
	}

	std::cout << std::endl;
	return x;
}

void Polygon::addFirstEdge(ofVec2f &s, ofVec2f &e) {
	if (!edges.empty()) {
		std::cout << "Error! Dangling edge cannot be added!" << std::endl;
		return;
	}
	makeAddEdge(s, e);
}

void Polygon::addEdge(ofVec2f &v) {
	if (edges.empty()) {
		std::cout << "Error! Dangling vertex cannot define edge!" << std::endl;
		return;
	}

	uint8_t n = static_cast<uint8_t>(edges.size());
	if (n == sides - 1) {
		closePolygon();
	}
	else {
		makeAddEdge(edges.back().end, v);
	}

}

void Polygon::closePolygon() {
	if (edges.empty()) {
		std::cout << "No edges found. Polygon cannot be closed!" << std::endl;
		return;
	}
	makeAddEdge(edges.back().end, edges.front().start);
	closed = true;
	std::cout << "Polygon closed!" << std::endl;
}

bool Polygon::noCollision(const ofVec2f &v) const {
	// std::cout << "Checking polygon collision." << std::endl;
	try {
		if (!closed) {
			throw std::logic_error("Under defined edges. Add more edges using `addEdge(Vertex)`.");
		}
		for (Edge e : edges) {
			if (!e.onLeft(v)) {
				// std::cout << "Returning true" << std::endl;
				return true;
			}
		}
		// std::cout << "Returning false" << std::endl;
		return false;
	}
	catch (std::logic_error &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return false;
	}
}

bool Polygon::noLineCollision(const ofVec2f& start, const ofVec2f& end) const {
	// int edge = 0;
	for (Edge e : edges) {
		if (e.is_intersecting(start, end)) {
			// std::cout << "edge: " << edge++ << " colliding" << std::endl;
			return false;
		}
		// std::cout << "edge: " << edge++ << " not colliding" << std::endl;
	}
	return true;
}

void Polygon::makeAddEdge(ofVec2f &s, ofVec2f &e) {
	edges.push_back(Edge(s, e));
	std::cout << "Edge added!" << std::endl;
}

void Polygon::checkPolygon() {
	try {
		polyErr();
	}
	catch (const std::domain_error& e) {
		std::cerr << "Error: " << e.what() << std::endl;
	}
	catch (const std::logic_error& e) {
		std::cerr << "Error: " << e.what() << std::endl;
	}
}

void Polygon::polyErr() {
	if (!obstacle_type.empty()) {
		uint8_t n = edges.size();
		if (n > sides) {
			std::string s = "More than " + std::to_string(sides) + " edges defined for the " + obstacle_type;
			throw std::domain_error(s);
		}
		else if (n < sides) {
			string s = "Under-defined polygon. Define " + to_string(sides - edges.size()) + "Use addEdge(Vertex) to add more edges.";
			throw std::logic_error(s);
		}
	}
}

// ********* Conic *********
Conic::Conic(ofVec2f v, double a, double b, const std::string &name): Obstacle(name), center(v), a(a), b(b) {
	std::cout << "Conic: " << std::endl;
}

bool Conic::noCollision(const ofVec2f &v) const {
	// std::cout << "Checking conic collision." << std::endl;
	double dx = (v.x - center.x) / a;
	double dy = (v.y - center.y) / b;

	return dx * dx + dy * dy > 1;
}

bool Conic::noLineCollision(const ofVec2f &s, const ofVec2f &e) const {
	if (not noCollision(s) or not noCollision(e)) {
		return false;
	}
	double m1 = (e.y - s.y) / (e.x - s.x);
	double denom = sqrt(a * a * m1 * m1 + b * b);
	double t1 = m1 * a * a / denom, t2 = b * b / denom;
	ofVec2f start(center.x - t1, center.y + t2);
	ofVec2f end(center.x + t1, center.y - t2);
	// std::cout << "(" << start.x << ", " << start.y << ") (" << end.x << ", " << end.y << ")" << std::endl;

	// double m2 = -b * b / (m1 * a * a); 8

	// calculate constant c1 and c2
	// double c1 = start.y - m1 * start.x, c2 = s.y - m2 * s.x;

	// calculate intersection points
	double d = (end.x - start.x) * (e.y - s.y) - (e.x - s.x) * (end.y - start.y), u1 = end.x * start.y - start.x * end.y, u2 = e.x * s.y - s.x * e.y;
	double x = ((e.x - s.x) * u1 - (end.x - start.x) * u2) / d;
	double y = ((e.y - s.y) * u1 - (end.y - start.y) * u2) / d;
	// bool ans = is_between(s, e, x, y) and is_between(start, end, x, y);
	// std::cout << "intersection point (" << x << ", " << y << ") between lines defined by l1: (" << start.x << ", " << start.y << ") (" << end.x << ", " << end.y << ") and l2: (" << s.x << ", " << s.y << ") (" << e.x  << ", " << e.y << ")";
	// if (ans) {
	// 	std::cout << "lines are intersecting!" << std::endl;
	// }
	// else {
	// 	std::cout << "lines are not intersecting!" << std::endl;
	// }
	// check if the intersecting point lies on the line segment 1 and 2
	return not(is_between(s, e, x, y) and is_between(start, end, x, y));
}

bool Conic::is_between(const ofVec2f& s, const ofVec2f& e, double x, double y) const {
	// round thevalues upto 2 decimal places
	x = std::round(x * 100) / 100.0;
	y = std::round(y * 100) / 100.0;
	// bool ans = x <= std::max(s.x, e.x) and x >= std::min(s.x, e.x) and y <= std::max(s.y, e.y) and y >= std::min(s.y, e.y);
	// if (ans) {
	// 	std::cout << "is between!" << std::endl;
	// }
	// else {
	// 	std::cout << "is not between!" << std::endl;
	// }
	// return ans;
	return x <= max(s.x, e.x) and x >= min(s.x, e.x) and y <= max(s.y, e.y) and y >= min(s.y, e.y);
}

Conic::~Conic() {
	std::cout << "Conic removed." << std::endl;
}

// ********* Triangle *********
Triangle::Triangle(const std::vector<ofVec2f> &v): Polygon(shoelace(const_cast<std::vector<ofVec2f>&>(v)), "Triangle") {
	sides = 3;
	checkPolygon();
	std::cout << "Triangle!" << std::endl;
}

Triangle::~Triangle() {
	std::cout << "Triangle removed." << std::endl;
}

// ********* Quadrilateral *********
Quadrilateral::Quadrilateral(const std::vector<ofVec2f> &v, const string &name): Polygon(shoelace(const_cast<std::vector<ofVec2f>&>(v)), name) {
	sides = 4;
	checkPolygon();
	std::cout << "Quadrilateral: " << std::endl;
}

Quadrilateral::~Quadrilateral() {}

// ********* Rectangle *********
Rectangle::Rectangle(ofVec2f &p, double l, double b, const string &name):
	Quadrilateral(std::vector<ofVec2f> { ofVec2f(p.x - l / 2, p.y - b / 2), ofVec2f(p.x - l / 2, p.y + b / 2), ofVec2f(p.x + l / 2, p.y + b / 2), ofVec2f(p.x + l / 2, p.y - b / 2)}, name) {
	std::cout << "Rectangle!" << std::endl;
}

Rectangle::~Rectangle() {
	std::cout << "Rectangle removed." << std::endl;
}

// ********* Square *********
Square::Square(ofVec2f &p, double l): Rectangle(p, l, l, "Square") {
	std::cout << "Square!" << std::endl;
}

Square::~Square() {
	std::cout << "Square removed." << std::endl;
}

// ********* Ellipse *********
Ellipse::Ellipse(ofVec2f &p, double a, double b, const string &name): Conic(p, a, b, name) {
	std::cout << "Ellipse!" << std::endl;
}

Ellipse::~Ellipse() {
	std::cout << "Ellipse removed." << std::endl;
}

// ********* Circle *********
Circle::Circle(ofVec2f &p, double r): Ellipse(p, r, r, "Circle") {
	std::cout << "Circle!" << std::endl;
}

Circle::~Circle() {
	std::cout << "Circle removed." << std::endl;
}

// ********* Map *********
Map::Map(uint16_t w, uint16_t h): width(w), height(h) {

	// The vertex of boundary are in clockwise direction
	// so that the region outside the boundary is is trated as an obstacle zone

	ofVec2f center(0, 0);
	std::vector<ofVec2f> v{ofVec2f(0, 0), ofVec2f(width, 0), ofVec2f(width, height), ofVec2f(0, height)};
	Obstacle *r = new Polygon(v);
	std::cout << "Screen size set to: " << w << "x" << h << std::endl;

	// the first obstacle describes the outer boundary
	addObstacle(r);
}

void Map::setup(std::vector<Obstacle *> &o) {
	// Insert the obstacles
	obstacles.insert(obstacles.end(), o.begin(), o.end());

	if (o.size() == 1) {
		// this indicates that the obstacle list passed is empty
		// map only consists of boundary
		std::cout << "Initiated an empty Map!" << std::endl;
	}
	else {
		std::cout << "Created a map!" << std::endl;
	}
}

void Map::addObstacle(Obstacle* o) {
	obstacles.push_back(o);
	std::cout << "Obstacle added!" << std::endl;
}

bool Map::noCollision(const ofVec2f& v) const {
	// int count = 0;
	for (Obstacle* o : obstacles) {
		// std::cout << "Checking Obstacle " << count++ << std::endl;
		if (not o->noCollision(v)) {
			return false;
		}
	}
	return true;
}

bool Map::noLineCollision(const ofVec2f &s, const ofVec2f &e) const {
	// int obs = 0;
	for (Obstacle* o : obstacles) {
		if (not o->noLineCollision(s, e)) {
			// std::cout << "Obstacle " << obs << ": colliding" << std::endl;
			return false;
		}
		// std::cout << "Obstacle " << obs++ << ": not colliding" << std::endl;
	}
	return true;
}


Map::~Map() {
	for (Obstacle *o : obstacles) {
		delete o;
	}
	std::cout << "Memory cleared..." << std::endl;
}