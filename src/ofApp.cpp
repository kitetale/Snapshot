#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    kinect.init();
    kinect.open();
    imitate(prevPx,kinect);
    imitate(imgDiff,kinect);
    
    drawptcloud = false;
    
    nearClip = 500; // in mm
    farClip = 1000; // in mm
    bucketNum = 6;
    bucketSize = 8000/bucketNum;
    
    colorImage.allocate(kinect.width,kinect.height);
    grayImage.allocate(kinect.width,kinect.height);
    grayBg.allocate(kinect.width,kinect.height);
    grayDiff.allocate(kinect.width,kinect.height);
    learnBg = false;
    grayThreshold = 30;
}

//--------------------------------------------------------------
void ofApp::update(){
    kinect.update();
    if (kinect.isFrameNew()){
        colorImage.setFromPixels(kinect.getDepthPixels());
        grayImage = colorImage;
        
        if (learnBg) {
            grayBg = grayImage;
            learnBg = false;
        }
        
        grayDiff.absDiff(grayBg, grayImage);
        grayDiff.threshold(grayThreshold);
        contourFinder.findContours(grayDiff, 30, (kinect.width*kinect.height), 10, true);
        
        
        /*
        absdiff(kinect,prevPx,imgDiff);
        imgDiff.update();
        copy(kinect,prevPx);
         */
        
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
        //grayImage.draw(0,0);
        colorImage.draw(0,0);
        contourFinder.draw(0,0);
    }
    int numBlobs = contourFinder.nBlobs;
    for (int i=0; i<numBlobs; i++){
       contourFinder.blobs[i].draw(360,540);
    }
    
    
}
//--------------------------------------------------------------
void ofApp::drawPointCloud(){
    // set modes to only draw points
    pointCloud.clear();
    //pointCloud.setMode(OF_PRIMITIVE_POINTS);
    pointIndex.clear();
    
    int h = kinect.height;
    int w = kinect.width;
    
    // point cloud creation
    for (int y=0; y<h; y++){
        for (int x=0; x<w; x++){
            ofVec3f point;
            point.set(0,0,0);
            // give me x y z pos in world at this vertex
            point = kinect.getWorldCoordinateAt(x,y);
            //if (point.z > nearClip && point.z < farClip){
                // add point to point cloud
                pointCloud.addVertex(point);
                
                // add color from rgb cam to each vertex
                //pointCloud.addColor(kinect.getColorAt(x,y));
                // add color from defined color space with z as hue

                ofColor color;
                color.setHsb(ofMap(point.z,100,8000,0,255), 255, 255);
                pointCloud.addColor(color);
                
            //}
            if (point.z){
                pointIndex.push_back(point.z); // valid point to make triangle mesh
            } else {
                pointIndex.push_back(0); // not valid point
            }
        }
    }
    
    // vertices into triangular mesh
    int th = 60;
    for (int y=0; y<h-1; y++){
        for (int x=0; x<w-1; x++){
            // check whether all three points are valid points
            if (abs(pointIndex[x+y*w]-pointIndex[(x+1)+y*w])<th &&
                abs(pointIndex[(x+1)+y*w]-pointIndex[x+((y+1)*w)])<th &&
                abs(pointIndex[x+((y+1)*w)]-pointIndex[x+y*w])<th &&
                pointIndex[x+y*w] &&
                pointIndex[(x+1)+y*w] &&
                pointIndex[x+((y+1)*w)]){
                // adding point, point below, point next to (triangle)
                pointCloud.addIndex(x+y*w);
                pointCloud.addIndex((x+1)+y*w);
                pointCloud.addIndex(x+((y+1)*w));
            }
            // check whether all three points are valid points
            if (abs(pointIndex[(x+1)+y*w]-pointIndex[(x+1)+((y+1)*w)])<th &&
                abs(pointIndex[(x+1)+((y+1)*w)]-pointIndex[x+((y+1)*w)])<th &&
                abs(pointIndex[x+((y+1)*w)]-pointIndex[(x+1)+y*w])<th &&
                pointIndex[(x+1)+y*w] &&
                pointIndex[(x+1)+((y+1)*w)] &&
                pointIndex[x+((y+1)*w)]){
                // adding point next to, point below, point prev to below
                // (opposite triangle to above)
                pointCloud.addIndex((x+1)+y*w);
                pointCloud.addIndex((x+1)+((y+1)*w));
                pointCloud.addIndex(x+((y+1)*w));
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
    //pointCloud.drawVertices();
    //pointCloud.drawWireframe();
    pointCloud.draw();
    
    // pop matrix when done
    ofPopMatrix();
    // disable depth test for other unrelated calls
    ofDisableDepthTest();
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
        
        // change threshold for contour drawing
        case 'n':
            ++grayThreshold;
            break;
        case 'm':
            --grayThreshold;
            break;
    
        // switch between different buckets
        case 'q':
            nearClip = 0;
            farClip = bucketSize;
            kinect.setDepthClipping(nearClip,farClip);
            break;
        case 'w':
            nearClip = bucketSize;
            farClip = bucketSize+bucketSize;
            kinect.setDepthClipping(nearClip,farClip);
            break;
        case 'e':
            nearClip = bucketSize+bucketSize;
            farClip = bucketSize*3;
            kinect.setDepthClipping(nearClip,farClip);
            break;
        case 'r':
            nearClip = bucketSize*3;
            farClip = bucketSize*4;
            kinect.setDepthClipping(nearClip,farClip);
            break;
        case 't':
            nearClip = bucketSize*4;
            farClip = bucketSize*5;
            kinect.setDepthClipping(nearClip,farClip);
            break;
        case 'y':
            nearClip = bucketSize*5;
            farClip = bucketSize*6;
            kinect.setDepthClipping(nearClip,farClip);
            break;
        case 'u':
            nearClip = bucketSize*6;
            farClip = 8000;
            kinect.setDepthClipping(nearClip,farClip);
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
