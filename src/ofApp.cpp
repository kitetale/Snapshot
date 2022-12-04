#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    kinect.init();
    kinect.open();
    imitate(prevPx,kinect);
    imitate(imgDiff,kinect);
    
    drawptcloud = false;
    
    h = kinect.height;
    w = kinect.width;
    
    nearClip = 100; // in mm
    farClip = 2500; // in mm
    kinect.setDepthClipping(nearClip,farClip);
    bucketNum = 6;
    bucketSize = 8000/bucketNum;
    
    colorImage.allocate(kinect.width,kinect.height);
    grayImage.allocate(kinect.width,kinect.height);
    grayBg.allocate(kinect.width,kinect.height);
    grayDiff.allocate(kinect.width,kinect.height);
    
    bucketImgGray.allocate(kinect.width,kinect.height);
    bucketImg.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    finalImg.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    
    //capturing images
    img0.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    img1.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    img2.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    img3.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    img4.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    img5.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    img6.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    img7.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    
    learnBg = false;
    grayThreshold = 30;
    
    curBucket = 10;
    
    // timestamp
    year = ofGetYear();
    month = ofGetMonth();
    day = ofGetDay();
    hr = ofGetHours();
    min = ofGetMinutes();
    sec = ofGetSeconds();
    lastMin = ofGetMinutes();
    timeGap = 1; // auto-capture frame every timeGap minutes
    captureIndex = 0;
}

//--------------------------------------------------------------
void ofApp::update(){
    kinect.update();
    if (kinect.isFrameNew()){
        colorImage.setFromPixels(kinect.getPixels());
        grayImage.setFromPixels(kinect.getDepthPixels());
        
        if (learnBg) {
            grayBg = grayImage;
            learnBg = false;
        }
        
        grayDiff.absDiff(grayBg, grayImage);
        grayDiff.threshold(grayThreshold);
        bucketImgGray = bucketImg;
        contourFinder.findContours(bucketImgGray, 30, (kinect.width*kinect.height), 10, true);
        
        /*
        absdiff(kinect,prevPx,imgDiff);
        imgDiff.update();
        copy(kinect,prevPx);
         */
        
    }
    
    //update time
    hr = ofGetHours();
    min = ofGetMinutes();
    sec = ofGetSeconds();
    if (((min-lastMin) == timeGap) || (min+60-lastMin == timeGap)) {
        autoCapture();
        lastMin = min;
    }
}
//--------------------------------------------------------------
void ofApp::autoCapture(){
    //use precomputed data
    ofPixels pix;
    pix.allocate(bucketImg.getWidth(), bucketImg.getHeight(), OF_IMAGE_QUALITY_BEST);
    pix = bucketImg.getPixels();
    ofSaveImage(pix,"bucket"+std::to_string(curBucket+1)+"/"+std::to_string(captureIndex)+".png");

    
    // I prob could have mod this part for style, but maybe not at 4am..
    unsigned char* img0_data = img0.getPixels().getData();
    unsigned char* img1_data = img1.getPixels().getData();
    unsigned char* img2_data = img2.getPixels().getData();
    unsigned char* img3_data = img3.getPixels().getData();
    unsigned char* img4_data = img4.getPixels().getData();
    unsigned char* img5_data = img5.getPixels().getData();
    unsigned char* img6_data = img6.getPixels().getData();
    unsigned char* img7_data = img7.getPixels().getData();
    for (int y=0; y<h; ++y){
        for (int x=0; x<w; ++x){
            ofVec3f point;
            point = kinect.getWorldCoordinateAt(x,y);
            int index = point.z>8000||point.z<=0?
                        10:
                            point.z<100&&point.z>0?
                            0:
                            floor(point.z/8000*bucketNum);
            if (index == curBucket) continue;
            switch (index){
                case '0':
                    img0_data[x+y*w] = 255;
                    img1_data[x+y*w] = 0;
                    img2_data[x+y*w] = 0;
                    img3_data[x+y*w] = 0;
                    img4_data[x+y*w] = 0;
                    img5_data[x+y*w] = 0;
                    img6_data[x+y*w] = 0;
                    img7_data[x+y*w] = 0;
                    break;
                case '1':
                    img0_data[x+y*w] = 0;
                    img1_data[x+y*w] = 255 - 50;
                    img2_data[x+y*w] = 0;
                    img3_data[x+y*w] = 0;
                    img4_data[x+y*w] = 0;
                    img5_data[x+y*w] = 0;
                    img6_data[x+y*w] = 0;
                    img7_data[x+y*w] = 0;
                    break;
                case '2':
                    img0_data[x+y*w] = 0;
                    img1_data[x+y*w] = 0;
                    img2_data[x+y*w] = 255 - 50 - 50;
                    img3_data[x+y*w] = 0;
                    img4_data[x+y*w] = 0;
                    img5_data[x+y*w] = 0;
                    img6_data[x+y*w] = 0;
                    img7_data[x+y*w] = 0;
                    break;
                case '3':
                    img0_data[x+y*w] = 0;
                    img1_data[x+y*w] = 0;
                    img2_data[x+y*w] = 0;
                    img3_data[x+y*w] = 255 - 50*3;
                    img4_data[x+y*w] = 0;
                    img5_data[x+y*w] = 0;
                    img6_data[x+y*w] = 0;
                    img7_data[x+y*w] = 0;
                    break;
                case '4':
                    img0_data[x+y*w] = 0;
                    img1_data[x+y*w] = 0;
                    img2_data[x+y*w] = 0;
                    img3_data[x+y*w] = 0;
                    img4_data[x+y*w] = 255 - 50*4;
                    img5_data[x+y*w] = 0;
                    img6_data[x+y*w] = 0;
                    img7_data[x+y*w] = 0;
                    break;
                case '5':
                    img0_data[x+y*w] = 0;
                    img1_data[x+y*w] = 0;
                    img2_data[x+y*w] = 0;
                    img3_data[x+y*w] = 0;
                    img4_data[x+y*w] = 0;
                    img5_data[x+y*w] = 255 - 50*5;
                    img6_data[x+y*w] = 0;
                    img7_data[x+y*w] = 0;
                    break;
                case '6':
                    img0_data[x+y*w] = 0;
                    img1_data[x+y*w] = 0;
                    img2_data[x+y*w] = 0;
                    img3_data[x+y*w] = 0;
                    img4_data[x+y*w] = 0;
                    img5_data[x+y*w] = 0;
                    img6_data[x+y*w] = 255 - 50*6;
                    img7_data[x+y*w] = 0;
                    break;
                case '7':
                    img0_data[x+y*w] = 0;
                    img1_data[x+y*w] = 0;
                    img2_data[x+y*w] = 0;
                    img3_data[x+y*w] = 0;
                    img4_data[x+y*w] = 0;
                    img5_data[x+y*w] = 0;
                    img6_data[x+y*w] = 0;
                    img7_data[x+y*w] = 255 - 50*7;
                    break;
                default:
                    img0_data[x+y*w] = 0;
                    img1_data[x+y*w] = 0;
                    img2_data[x+y*w] = 0;
                    img3_data[x+y*w] = 0;
                    img4_data[x+y*w] = 0;
                    img5_data[x+y*w] = 0;
                    img6_data[x+y*w] = 0;
                    img7_data[x+y*w] = 0;
                    break;
            }
            
        }
    }
    img0.update();
    img1.update();
    img2.update();
    img3.update();
    img4.update();
    img5.update();
    img6.update();
    img7.update();
    
    ofPixels pix0, pix1, pix2, pix3, pix4, pix5, pix6, pix7;
    pix0.allocate(img0.getWidth(), img0.getHeight(), OF_IMAGE_QUALITY_BEST);
    pix1.allocate(img1.getWidth(), img1.getHeight(), OF_IMAGE_QUALITY_BEST);
    pix2.allocate(img2.getWidth(), img2.getHeight(), OF_IMAGE_QUALITY_BEST);
    pix3.allocate(img3.getWidth(), img3.getHeight(), OF_IMAGE_QUALITY_BEST);
    pix4.allocate(img4.getWidth(), img4.getHeight(), OF_IMAGE_QUALITY_BEST);
    pix5.allocate(img5.getWidth(), img5.getHeight(), OF_IMAGE_QUALITY_BEST);
    pix6.allocate(img6.getWidth(), img6.getHeight(), OF_IMAGE_QUALITY_BEST);
    pix7.allocate(img7.getWidth(), img7.getHeight(), OF_IMAGE_QUALITY_BEST);
    
    pix0 = img0.getPixels();
    pix1 = img1.getPixels();
    pix2 = img2.getPixels();
    pix3 = img3.getPixels();
    pix4 = img4.getPixels();
    pix5 = img5.getPixels();
    pix6 = img6.getPixels();
    pix7 = img7.getPixels();
    
    ofPixels thisPix;
    for (int i=0; i<bucketNum; ++i){
        if (i==curBucket) continue;
        
        switch (i){
            case '0': thisPix = pix0; break;
            case '1': thisPix = pix1; break;
            case '2': thisPix = pix2; break;
            case '3': thisPix = pix3; break;
            case '4': thisPix = pix4; break;
            case '5': thisPix = pix5; break;
            case '6': thisPix = pix6; break;
            default: thisPix = pix7; break;
        }
        
        ofSaveImage(thisPix,"bucket"+std::to_string(i+1)+"/"+std::to_string(captureIndex)+".png");
    }
    
    ++captureIndex;
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
        unsigned char* bucketImgData = bucketImg.getPixels().getData();
        unsigned char* finalImgData = finalImg.getPixels().getData();
        //std::cout<<"start:==========================================="<<std::endl;
        for (int y=0; y<h; ++y){
            for (int x=0; x<w; ++x){
                ofVec3f point;
                point = kinect.getWorldCoordinateAt(x,y);
                //std::cout<<"x: "<<x<<", y: "<<y<<", z: "<<z<<std::endl;
                int index = point.z>8000||point.z<=0?
                            10:
                                point.z<100&&point.z>0?
                                0:
                                floor(point.z/8000*bucketNum);
    
                finalImgData[x+y*w] = 255 - 50*index;
                if (index == curBucket){
                    bucketImgData[x+y*w] = 255 - 50*index;
                } else {
                    bucketImgData[x+y*w] = 0;
                }
            }
        }
        //std::cout<<"end ==========================================="<<std::endl;
        bucketImg.update();
        finalImg.update();

        grayImage.draw(0,0, kinect.width/2,kinect.height/2);
        colorImage.draw(kinect.width/2,0,kinect.width/2,kinect.height/2);
        bucketImg.draw(0,kinect.height/2,kinect.width/2,kinect.height/2);
        contourFinder.draw(kinect.width/2,kinect.height/2,kinect.width/2,kinect.height/2);
        finalImg.draw(0,kinect.height,kinect.width/2,kinect.height/2);
        
        // draw time on screen
        ofDrawBitmapString(std::to_string(hr)+":"+std::to_string(min)+":"+std::to_string(sec), kinect.width/2, kinect.height*1.5);
    }
    
    
}
//--------------------------------------------------------------
void ofApp::drawPointCloud(){
    // set modes to only draw points
    pointCloud.clear();
    bucketCloud.clear();
    
    //pointCloud.setMode(OF_PRIMITIVE_POINTS);
    pointIndex.clear();
    
    bucketIndex.clear();
    
    // point cloud creation
    for (int y=0; y<h; ++y){
        for (int x=0; x<w; ++x){
            ofVec3f point;
            point.set(0,0,0);
            // give me x y z pos in world at this vertex
            point = kinect.getWorldCoordinateAt(x,y);
            
            // add point to point cloud
            pointCloud.addVertex(point);
            // add color from rgb cam to each vertex
            //pointCloud.addColor(kinect.getColorAt(x,y));
            // add color from defined color space with z as hue
            ofColor color;
            color.setHsb(ofMap(point.z,100,8000,0,255), 255, 255);
            pointCloud.addColor(color);
            
            bucketCloud.addVertex(point);
            bucketCloud.addColor(255);
            
            
            int index = point.z>8000?
                        bucketNum-1:
                            point.z<100?
                            0:
                            floor(point.z/8000*bucketNum);
            
            
            if (point.z){
                bucketIndex.push_back(index);
                pointIndex.push_back(point.z); // valid point to make triangle mesh
            } else {
                bucketIndex.push_back(-1);
                pointIndex.push_back(0); // not valid point
            }
        }
    }
    
    // vertices into triangular mesh
    int th = 60;
    for (int y=0; y<h-1; ++y){
        for (int x=0; x<w-1; ++x){
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
            
            // if points in same buckets, add index
            if (curBucket==bucketIndex[x+y*w] &&
                bucketIndex[x+y*w]==bucketIndex[(x+1)+y*w] &&
                bucketIndex[x+((y+1)*w)]==bucketIndex[(x+1)+y*w] &&
                bucketIndex[x+((y+1)*w)]==bucketIndex[x+y*w]){
                bucketCloud.addIndex(x+y*w);
                bucketCloud.addIndex((x+1)+y*w);
                bucketCloud.addIndex(x+((y+1)*w));
            }
            // other side of triangle
            if (curBucket==bucketIndex[(x+1)+y*w] &&
                bucketIndex[(x+1)+y*w]==bucketIndex[(x+1)+((y+1)*w)] &&
                bucketIndex[x+((y+1)*w)]==bucketIndex[(x+1)+((y+1)*w)] &&
                bucketIndex[x+((y+1)*w)]==bucketIndex[(x+1)+y*w]){
                bucketCloud.addIndex((x+1)+y*w);
                bucketCloud.addIndex((x+1)+((y+1)*w));
                bucketCloud.addIndex(x+((y+1)*w));
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
    if (curBucket<8){
        bucketCloud.draw();
    } else {
        pointCloud.draw();
    }
        
        
    
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
        case 'z':
            if (bucketNum < 7) {
                ++bucketNum;
                bucketSize = 8000/bucketNum;
            }
            break;
        case 'x':
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
            curBucket = 0;
            break;
        case 'w':
            nearClip = bucketSize;
            farClip = bucketSize+bucketSize;
            kinect.setDepthClipping(nearClip,farClip);
            curBucket = 1;
            break;
        case 'e':
            nearClip = bucketSize+bucketSize;
            farClip = bucketSize*3;
            kinect.setDepthClipping(nearClip,farClip);
            curBucket = 2;
            break;
        case 'r':
            nearClip = bucketSize*3;
            farClip = bucketSize*4;
            kinect.setDepthClipping(nearClip,farClip);
            curBucket = 3;
            break;
        case 't':
            nearClip = bucketSize*4;
            farClip = bucketSize*5;
            kinect.setDepthClipping(nearClip,farClip);
            curBucket = 4;
            break;
        case 'y':
            nearClip = bucketSize*5;
            farClip = bucketSize*6;
            kinect.setDepthClipping(nearClip,farClip);
            curBucket = 5;
            break;
        case 'u':
            nearClip = bucketSize*6;
            farClip = 8000;
            kinect.setDepthClipping(nearClip,farClip);
            curBucket = 6;
            break;
            
        // save contour image
        case 's':
        {
            ofPixels pix;
            pix.allocate(bucketImg.getWidth(), bucketImg.getHeight(), OF_IMAGE_QUALITY_BEST);
            pix = bucketImg.getPixels();
            ofSaveViewport("mySnapshot.png");
            break;
        }
            
        default:
            nearClip = 100; // in mm
            farClip = 2500; // in mm
            kinect.setDepthClipping(nearClip,farClip);
            curBucket = 10;
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
