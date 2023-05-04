#pragma once

#include "ofMain.h"
#include "rrt_star.hpp"

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	void recDraw(rrtNode *node);
private:
	Map m;
	Map dyn_m;
	rrtStar rrt;
	// ofVec2f s{1000, 650}, e{800, 500};

};
