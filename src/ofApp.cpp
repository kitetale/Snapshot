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
    //imgDiff.draw(0,0);
    
    for (int i = 0; i < kinect.getHeight(); i+=8){
        ofPolyline polyline;
        for (int j = 0; j < kinect.getWidth(); j++){
            ofColor col = imgDiff.getPixels().getColor(j,i);
            int brightness = col.getBrightness();
            polyline.addVertex(j, i+ofMap(brightness, 0, 255, 0, -64));
            //sends y coord up if bright
        }
        polyline = polyline.getSmoothed(10);
        polyline.draw();
    }
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
