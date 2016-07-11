#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

struct TeleGPS {
    bool    fix;
    int     quality;
    int     satelites;
    
    float lat;
    float lon;
};

struct TeleIMU {
    float yaw;
    float pitch;
    float roll;
};

#endif // __TELEMETRY_H__
