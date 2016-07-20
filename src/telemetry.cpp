#include "telemetry.h"


void TeleGPS::init() {
    red.load("res/GPS-red.png");
    white.load("res/GPS-white.png");
}

void TeleGPS::draw(int xp, int yp) {
    if(fix) {
        white.draw(xp, yp, 32, 32);
        ofSetColor(255, 255, 255);
    } else {
        red.draw(xp, yp, 32, 32);
        ofSetColor(255, 0, 0);
    }
    
    stringstream ss;
    ss << "sats: " << satelites;
    ofDrawBitmapString(ss.str(), xp-20, yp+50);
}

void Telemetry::init() {
    gps.init();
}

int Telemetry::parse(string line) {
    //    ofLogNotice() << "onNewSerialLine, message: " << line << endl;
    
    vector<string> input = ofSplitString(line, ",");
    string msgtype = input.at(0);
    
    if(msgtype == "GPSL") {
        if( input.size() == 3) {
            ofLogNotice() << "GPSL line found";
            gps.lat = ofToFloat( input.at(1) );
            gps.lon = ofToFloat( input.at(2) );
            int i = 0;
            ofNotifyEvent( onGpsCoordinates, i );
        } else {
            ofLogNotice() << "ignored GPS reading: " << line;
        }
    } else if (msgtype == "GPSQ") {
        if( input.size() == 4) {
            ofLogNotice() << "GPSQ line found";
            gps.fix = ofToBool( input.at(1) );
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

