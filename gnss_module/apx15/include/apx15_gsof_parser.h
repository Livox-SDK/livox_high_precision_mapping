#ifndef APX15GSOFPARSER_H
#define APX15GSOFPARSER_H

namespace apx15
{

#define PI (3.14159265358979)
#define PI2DEGREE (57.295779513082323)
#define DEGREE2PI (0.017453292519943)

#if UINT_MAX == 0xFFFFFFFF
    typedef unsigned int U32;
#else
    typedef unsigned long U32;
#endif

typedef unsigned short U16;
typedef signed short S16;
typedef unsigned char U8;
typedef double DBL;
typedef float FLT;
typedef signed long S32;

#define PRINT_DATA  0


class apx15Gsof {
public:
    apx15Gsof();
    ~apx15Gsof();
    void openLogParsedDataFile(void);
    unsigned char grabGsof(unsigned char * pData, int length);

    unsigned char gsof31Hreceived;
    U16 weekNum;
    U32 gpsTimeInMilliSec;
    S16 leapSecond;
    bool leapSecondReceived = false;
    U8 IMU_AlignmentStatus;
    U8 GPS_Quality;
    DBL latitude;
    DBL longitude;
    DBL altitude;

    FLT northVelocity;
    FLT eastVelocity;
    FLT downVelocity;

    FLT totalSpeed;

    DBL roll;
    DBL pitch;
    DBL yaw;

    DBL trackAngle;

    FLT angularRate[3];
    FLT acceleration[3] ;


    FLT northPositionRMS;
    FLT eastPositionRMS;
    FLT downPositionRMS;

    FLT northVelocityRMS;
    FLT eastVelocityRMS;
    FLT downVelocityRMS;

    DBL rollRMS;
    DBL pitchRMS;
    DBL yawRMS;

private:
    U32 getU32( unsigned char * * ppData );
    float getFloat( unsigned char * * ppData );
    double getDouble( unsigned char * * ppData );
    unsigned short getU16( unsigned char * * ppData );
    S16 getS16(unsigned char * * ppData);
    S32 getS32( unsigned char * * ppData );
    void processEventMarker( int length, unsigned char *pData );
    void processINSNavigationInfo( int length, unsigned char *pData );
    void processINSRMSInfo(int length, unsigned char *pData);
    void processUTC(int length, unsigned char *pData);
    void processGsofData( void );
    void postGsofData( unsigned char * pData, int length );


    // A few global variables needed for collecting full GSOF packets from
    // multiple Trimcomm packets.
    unsigned char gsofData[2048];
    int gsofDataIndex;
};


}//namespace apx15


#endif // APX15GSOFPARSER_H
