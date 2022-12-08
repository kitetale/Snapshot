#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxThermalPrinter.h"

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
    void autoCapture();
    void updateLayers();
    void makeSnapshot();
    void printSnapshot();
    void belowText();

	ofxKinect kinect;
    ofImage imgDiff;
    ofPixels prevPx;
    
	int angle;
    bool drawptcloud;
    int nearClip, farClip;
    int bucketSize;
    int bucketNum;
    
    int h,w;
    
    ofEasyCam cam; // allows to look around the point cloud in 3D space
    
    ofxCvColorImage colorImage;
    ofxCvGrayscaleImage grayImage, grayBg, grayDiff;
    ofxCvContourFinder contourFinder;
    bool learnBg;
    int grayThreshold;
    
    // point clouds
    ofMesh pointCloud; // all vertices
    ofMesh bucketCloud; // all vertices
    vector<int> pointIndex; // all data
    vector<int> bucketIndex; // for current bucket data
    
    // current layer
    int curBucket; //current bucket index
    ofImage bucketImg, finalImg; //ofImage for saving to os
    ofxCvGrayscaleImage bucketImgGray; //grayscale for contourfinder
    
    // for capturing layers
    ofImage img0, img1, img2, img3, img4, img5, img6, img7;
    
    //timestamps
    int year,month,day,hr,minutes,sec;
    int lastMin, lastSec;
    int timeGap;
    int captureIndex, snapshotIndex;
    vector<string> captureTime; // "mm/dd/yy hr:min:sec"
    string startT;
    string endT;
    ofImage output; // output of snapshot
    
    ofTrueTypeFont font;
    ofTrueTypeFont font2;
    
    ofxThermalPrinter printer;
    bool viewCurrent;
};
