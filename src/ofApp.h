#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
//#include "ofxOSXBoost.h"
#include "ofxXmlSettings.h"
#include "ofxLiDAR.h"
#include "ofxSimpleSerial.h"
#include "telemetry.h"

// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
    void savePointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
    
    void onNewSerialLine(string &message);
	
	ofxKinect kinect;
    //ofxLiDARRecorder recorder;

    ofxSimpleSerial	serial;
    string serline;
    
    ofxXmlSettings settings;
    
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
    
    TeleGPS gps;
    TeleIMU imu;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
    
    bool bSavePointCloud;
	
	int nearThreshold;
	int farThreshold;
    
    int iSaveIndex;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
};
