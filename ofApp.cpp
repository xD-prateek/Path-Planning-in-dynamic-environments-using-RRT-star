#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	// MIND THE ORDER OF VERTEX
	// The obstacle area is present on the left side of the edge

	/*
	OBSTACLE 1:
		Rectangle -> Center: 500, 225
					 Length: 1000
					 Bredth: 100
	*/
	ofVec2f c(500, 225);
	Obstacle *r = new Rectangle(c, 1000, 100);
	m.addObstacle(r);

	/*
	OBSTACLE 2:
		Square -> Center: 1440, 160
				  Side: 320
	*/
	c.set(1520, 80);
	r = new Square(c, 160);
	m.addObstacle(r);

	/*
	OBSTACLE 3:
		Ellipse -> Center: 690, 500
				   major_axis: 200
				   minor_axis:100
	*/
	c.set(690, 500);
	r = new Ellipse(c, 200, 100);
	m.addObstacle(r);

	/*
	OBSTACLE 4:
		Triangle -> Point1: 650, 850
					Point2: 850, 690
					Point3: 1100, 800
	*/
	ofVec2f t1(650, 850), t2(850, 690), t3(1100, 800);
	std::vector<ofVec2f> a{t1, t2, t3};
	r = new Triangle(a);
	m.addObstacle(r);

	/*
	OBSTACLE 5:
		Circle -> Center: 1300, 450
				  Radius: 100
	*/
	c.set(1300, 450);
	r = new Circle(c, 100);
	m.addObstacle(r);

	/*
	OBSTACLE 6:
		Polygon -> Point1: 1500, 800
				   Point2: 1550, 760
				   Point3: 1600, 820
				   Point4: 1600, 900
				   Point5: 1520, ,900
				   Point6: 1450, 860
	*/
	ofVec2f p1(1500, 800), p2(1550, 760), p3(1600, 820), p4(1600, 900), p5(1520, 900), p6(1450, 860);
	std::vector<ofVec2f> b{p1, p2, p3, p4, p5, p6};
	r = new Polygon(b);
	m.addObstacle(r);

	/*
	OBSTACLE 7:
		Circle -> Center: 400, 300
				  Radius: 50
	*/
	c.set(800, 450);
	r = new Circle(c, 50);
	dyn_m.addObstacle(r);

	// initiate rrt star
	ofVec2f s(START_X, START_Y), e(END_X, END_Y);
	rrt.setMaps(&m, &dyn_m);
	rrt.setup(s, e);

	// if (m.noLineCollision(s, e)) {
	// 	std::cout << "Clear!!!" << std::endl;
	// }
	// else {
	// 	std::cout << "Colliding!!!" << std::endl;
	// }
	// set background color
	ofBackground(77, 216, 247);

}

//--------------------------------------------------------------
void ofApp::update() {
	rrt.update();
	rrt.dynamicUpdate();

	ofSleepMillis(UPDATE_TIME * 1000);

	// for (rrtNode &node : rrt.nodes) {
	// 	if (node.parent) {
	// 		std::cout << "Parent of (" << node.p.x << ", " << node.p.y << ") is (" << (node.parent)->p.x << ", " << (node.parent)->p.y << ")" << std::endl;

	// 	}
	// }
}

//--------------------------------------------------------------
void ofApp::draw() {

	std::vector<Obstacle *> &o = m.obstacles, &dyn_o = dyn_m.obstacles;
	int n = static_cast<int>(o.size());
	int dyn_n = static_cast<int>(dyn_o.size());

	// set outer boundary
	ofSetColor(150);
	Polygon *b = dynamic_cast<Polygon *>(o[0]);
	ofBeginShape();
	for (Edge &e : b->edges) {
		ofVertex(e.end);
	}
	ofEndShape(true);

	// draw obstacles
	ofSetColor(77, 216, 247);
	for (int i = 1; i < n; ++i) {
		if (o[i]->obstacle_type == "Ellipse" or o[i]->obstacle_type == "Circle") {
			Conic *c = dynamic_cast<Conic *>(o[i]);
			ofDrawEllipse(c->center.x, c->center.y, 2 * c->a, 2 * c->b);
		}
		else {
			Polygon *p = dynamic_cast<Polygon *>(o[i]);
			ofBeginShape();
			for (Edge &e : p->edges) {
				ofVertex(e.end);
			}
			ofEndShape(true);
		}
	}

	// draw dynamic obstacles
	ofSetColor(219, 2, 89, 200);
	for (int i = 1; i < dyn_n; ++i) {
		if (dyn_o[i]->obstacle_type == "Ellipse" or dyn_o[i]->obstacle_type == "Circle") {
			Conic *c = dynamic_cast<Conic *>(dyn_o[i]);
			ofDrawEllipse(c->center.x, c->center.y, 2 * c->a, 2 * c->b);
		}
		else {
			Polygon *p = dynamic_cast<Polygon *>(dyn_o[i]);
			ofBeginShape();
			for (Edge &e : p->edges) {
				ofVertex(e.end);
			}
			ofEndShape(true);
		}
	}

	// draw rrt graph

	// start node and goal node
	ofSetColor(219, 247, 77, 120);
	ofDrawCircle(rrt.goal, GOAL_VICINITY);
	ofSetColor(0);
	// ofDrawLine(s, e);
	ofSetColor(0);
	ofDrawCircle(rrt.start->p, 10);
	ofDrawCircle(rrt.goal, 10);

	// graph edges
	ofSetLineWidth(2);
	// for (rrtNode *node : rrt.nodes) {
	// 	if (node->parent) {
	// 		ofDrawCircle(node->p, 5);
	// 		// std::cout << "Parent of (" << node.p.x << ", " << node.p.y << ") changed to (" << (node.parent)->p.x << ", " << (node.parent)->p.y << ")" << std::endl;
	// 		ofDrawLine(node->p, node->parent->p);
	// 	}
	// }

	recDraw(rrt.start);

	// draw the path
	ofSetColor(255, 138, 0);
	if (rrt.goal_reached) {
		ofSetLineWidth(5);
		rrtNode *node = rrt.goal_node;
		while (node->parent) {
			ofDrawCircle(node->p, 5);
			ofDrawLine(node->p, node->parent->p);
			node = node->parent;
		}
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	// switch (key) {
	// case ' ':
	// 	x = ofRandom(0, ofGetWidth());
	// 	y = ofRandom(0, ofGetHeight());
	// 	break;
	// default:
	// 	break;
	// }
	// if (key == ' ') {
	// 	x = ofRandom(0, ofGetWidth());
	// 	y = ofRandom(0, ofGetHeight());
	// }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
	ofVec2f p(x, y);
	if (not button and not dyn_m.noCollision(p)) {
		for (Obstacle* o : dyn_m.obstacles) {
			if (o->obstacle_type == "Ellipse" or o->obstacle_type == "Circle") {
				Conic *c = dynamic_cast<Conic *>(o);
				c->center.set(p);
			}
		}
	}
	else {
		if (hypot(p.x - rrt.goal.x, p.y - rrt.goal.y) < GOAL_VICINITY) {
			rrt.goal.set(p);
		}
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

void ofApp::recDraw(rrtNode *node) {
	for (rrtNode* child : node->children) {
		ofDrawLine(node->p, child->p);
		ofDrawCircle(child->p, 5);
		recDraw(child);
	}
}
