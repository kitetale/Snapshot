#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxGui.h"

using namespace ofxCv;
using namespace cv;

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

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
    
    void drawPointCloud();

	ofxKinect kinect;
    ofImage imgDiff;
    ofPixels prevPx;
	int angle;
    bool drawptcloud;
    int nearClip, farClip;
    int bucketSize;
    int bucketNum;
    
    ofEasyCam cam; // allows to look around the point cloud in 3D space
    
    ofxCvColorImage colorImage;
    ofxCvGrayscaleImage grayImage, grayBg, grayDiff;
    ofxCvContourFinder contourFinder;
    bool learnBg;
    int grayThreshold;
    
    ofMesh pointCloud;
    vector<int> pointIndex;
};
