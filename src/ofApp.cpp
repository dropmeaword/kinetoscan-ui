#include "ofApp.h"
#include <iomanip>

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
    
    ofLogNotice() << "Loading config from settings.xml";
    settings.loadFile("settings.xml");

    //ofLogNotice() << ofxLiDAR::getLibLASVersion();
    //recorder.create("lidar.las");
    
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
    string serialdev = settings.getValue("settings:serial", "/dev/tty.wchusbserial1410");
    int baudrate     = settings.getValue("settings:baudrate", 115200);
    serial.setup(serialdev, baudrate);
    if(serial.isInitialized()) {
        ofLogNotice() << "Serial was initialized ok, setting callback.";
        serial.startContinuousRead(false);
        ofAddListener(serial.NEW_MESSAGE, this, &ofApp::onNewSerialLine);
    }
    serline = "";
    
    gps.lat = settings.getValue("settings:lastKnownPosition:lat", 0.0);
    gps.lon = settings.getValue("settings:lastKnownPosition:lon", 0.0);
    
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = true;
    bSavePointCloud = false;
    
    iSaveIndex = 0;
    
    int capms = 1000 / SCAN_FREQUENCY_HZ;
    ofLogNotice() << "Capturing a 3D scan every " << capms << "ms";
    capture.setup( capms );
    capture.start(true);
    ofAddListener( capture.TIMER_COMPLETE , this, &ofApp::timerScanCallback );
}

void ofApp::timerScanCallback( int &args )
{
    bSavePointCloud = true;
    //ofLogNotice() << "Capturing 3D scan " << ofGetElapsedTimeMillis();
}

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
    capture.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);
		
		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
    }

//    ofEnableDepthTest();
//    glEnable(GL_CULL_FACE);
//    glCullFace(GL_BACK);
    
    // draw cube
    ofPushMatrix();
    ofTranslate(ofGetWidth() / 2, ofGetHeight() / 2, 300);
    ofBoxPrimitive(40,20,40).getMesh().drawWireframe();
    ofPopMatrix();
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
        
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
	ofDrawBitmapString(reportStream.str(), 20, 652);
    
}

void ofApp::onNewSerialLine(string &line)
{
//    ofLogNotice() << "onNewSerialLine, message: " << line << endl;

    vector<string> input = ofSplitString(line, ",");
    string msgtype = input.at(0);
    
    if(msgtype == "GPSL") {
        if( input.size() == 3) {
            ofLogNotice() << "GPSL line found";
            gps.lat = ofToFloat( input.at(1) );
            gps.lon = ofToFloat( input.at(2) );
        } else {
            ofLogNotice() << "ignored GPS reading: " << line;
        }
    } else if (msgtype == "GPSQ") {
        if( input.size() == 4) {
            ofLogNotice() << "GPSQ line found";
            gps.fix = ofToInt( input.at(1) );
            gps.quality = ofToInt( input.at(2) );
            gps.satelites = ofToInt( input.at(3) );
        } else {
            ofLogNotice() << "ignored GPS reading: " << line;
        }
    } else if(msgtype == "IMU") {
        if( input.size() == 4) {
            imu.yaw = ofToFloat( input.at(1) );
            imu.pitch = ofToFloat( input.at(2) );
            imu.roll = ofToFloat( input.at(3) );
        } else {
            ofLogNotice() << "rejected IMU reading: " << line;
        }

//        ofLogNotice() << "IMU line found";
//        for ( std::vector<std::string>::iterator it=input.begin(); it<input.end(); it++) {
//            std::cout << ' ' << *it;
//        }
    }
}


void ofApp::savePointCloud() {
    // save pintcloud
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
    
    stringstream ss;
    
    if(bSavePointCloud) {
        ss << "mesh-" << setfill('0') << setw(5) << iSaveIndex++ << ".ply";
        mesh.save(ss.str(), true);
        ofLogNotice() << "saved mesh to " << ss.str();
        bSavePointCloud = false;
    }
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;

        case's':
            bSavePointCloud = !bSavePointCloud;
            break;

		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}