#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxXmlSettings.h"
#include "ofxSimpleSerial.h"
#include "telemetry.h"
#include "ofxSimpleTimer.h"
#include "ofxGui.h"

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


#define SCAN_FREQUENCY_HZ 4   // how many scans per second do we do?


class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
    void initUI();
    void initKinect();

	void drawPointCloud();
    void savePointCloud();
    void drawComments();
    void drawLiveKinectFeed();
    
    void drawBoundingBox();
	
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
    
    Telemetry tele;
    
    ofxSimpleTimer capture;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
    
    bool bSavePointCloud;
    
    bool bHideUI;
	
	int nearThreshold;
	int farThreshold;
    
    int iSaveIndex;
	
	int angle;
    
    void cb_button_reset();
    void cb_timer_scan( int &args );
    void cb_gps_updated( int &args );
        
    ofxToggle scanning;
    ofxFloatSlider width;
    ofxFloatSlider height;
    ofxFloatSlider depth;
    ofxButton resetq;


//    ofxColorSlider color;
//    ofxVec2Slider center;
//    ofxIntSlider circleResolution;
//    ofxButton twoCircles;
//    ofxButton ringButton;
//    ofxLabel screenSize;
    
    ofxPanel gui;

    ofMesh mesh;
	
	// used for viewing the point cloud
	ofEasyCam cam;
};
