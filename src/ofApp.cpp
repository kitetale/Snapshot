#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    kinect.init();
    kinect.open();
    imitate(prevPx,kinect);
    imitate(imgDiff,kinect);
}

//--------------------------------------------------------------
void ofApp::update(){
    kinect.update();
    if (kinect.isFrameNew()){
        absdiff(kinect,prevPx,imgDiff);
        imgDiff.update();
        copy(kinect,prevPx);
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    //kinect.draw(0,0,640,480);
    imgDiff.draw(0,0);
}

//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(0);
    kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key){
    case OF_KEY_UP:
        if (angle < 30) {
            ++angle;
        }
        kinect.setCameraTiltAngle(angle);
        break;

    case OF_KEY_DOWN:
        if (angle > -30) {
            --angle;
        }
        kinect.setCameraTiltAngle(angle);
        break;
    
    default:
        break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
