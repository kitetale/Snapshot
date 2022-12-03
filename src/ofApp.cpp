#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    kinect.init();
    kinect.open();
    imitate(prevPx,kinect);
    imitate(imgDiff,kinect);
    
    drawptcloud = false;
    
    nearClip = 0; // in mm
    farClip = 8000; // in mm
    bucketNum = 6;
    bucketSize = 8000/bucketNum;
    
    gui.setup("near far clip panel");
    gui.add(nearclip.set("near clip",500,100,8000));
    gui.add(farclip.set("far clip",800,500,8000));
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
    //kinect.draw(0,0,kinect.width/2,kinect.height/2);
    //imgDiff.draw(0,kinect.height/2);
    /*
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
    */
    if (drawptcloud){
        cam.begin(); //allow to look around point cloud in 3D
        drawPointCloud();
        cam.end();
    } else {
        kinect.drawDepth(0,0);
    }
    
    gui.draw();
}
//--------------------------------------------------------------
void ofApp::drawPointCloud(){
    ofMesh pointCloud;
    // set modes to only draw points
    pointCloud.setMode(OF_PRIMITIVE_POINTS);
    
    for (int y=0; y<kinect.height; ++y){
        for (int x=0; x<kinect.width; ++x){
            ofVec3f point;
            // give me x y z pos in world at this vertex
            point = kinect.getWorldCoordinateAt(x,y);
            if (point.z > nearClip && point.z < farClip){
                // add point to point cloud
                pointCloud.addVertex(point);
                
                // add color from rgb cam to each vertex
                //pointCloud.addColor(kinect.getColorAt(x,y));
                // add color from defined color space with z as hue
                ofColor color;
                color.setHsb(ofMap(point.z,100,8000,0,255), 255, 255);
                pointCloud.addColor(color);
            }
        }
    }
    
    // define size of point drawn
    glPointSize(3);
    
    // enable depth test to draw points based on depth
    // (so that front drawn later than background)
    ofEnableDepthTest();
    // push matrix to not mess up matrix
    ofPushMatrix();
    // use scale to flip orientation of point cloud drawing (since coord goes down as drawn)
    ofScale(1,-1,-1);
    // translate back
    ofTranslate(0,0,-1000);
    pointCloud.drawVertices();
    
    // pop matrix when done
    ofPopMatrix();
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
               
        // change to point cloud mode
        case 'p':
            drawptcloud = !drawptcloud;
            break;
        
        // change bucket num
        case 'a':
            if (bucketNum < 8) {
                ++bucketNum;
                bucketSize = 8000/bucketNum;
            }
            break;
        case 's':
            if (bucketNum > 1) {
                --bucketNum;
                bucketSize = 8000/bucketNum;
            }
            break;
    
        // switch between different buckets
        case 'q':
            nearClip = 0;
            farClip = bucketSize;
            break;
        case 'w':
            nearClip = bucketSize;
            farClip = bucketSize+bucketSize;
            break;
        case 'e':
            nearClip = bucketSize+bucketSize;
            farClip = bucketSize*3;
            break;
        case 'r':
            nearClip = bucketSize*3;
            farClip = bucketSize*4;
            break;
        case 't':
            nearClip = bucketSize*4;
            farClip = bucketSize*5;
            break;
        case 'y':
            nearClip = bucketSize*5;
            farClip = bucketSize*6;
            break;
        case 'u':
            nearClip = bucketSize*6;
            farClip = 8000;
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
