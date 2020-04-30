#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <limits.h>

#include "apx15_gsof_parser.h"
#include "ros/console.h"


namespace apx15
{


apx15Gsof::apx15Gsof()
{

}

apx15Gsof::~apx15Gsof()
{

}


void apx15Gsof::openLogParsedDataFile(void)
{
    //pFile = fopen ("myfile.txt","w");
}

/**********************************************************************/
U32 apx15Gsof::getU32(unsigned char * * ppData)
/**********************************************************************/
// Used by the decoding routines to grab 4 bytes and pack them into
// a U32.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
{
  U32 retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 3 ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getU32() */



/**********************************************************************/
float apx15Gsof::getFloat(unsigned char * * ppData)
/**********************************************************************/
// Used by the decoding routines to grab 4 bytes and pack them into
// a Float.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  float retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 3 ;


  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getFloat() */



/**********************************************************************/
double apx15Gsof::getDouble(unsigned char * * ppData)
/**********************************************************************/
// Used by the decoding routines to grab 8 bytes and pack them into
// a Double.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  double retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 7 ;


  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getDouble() */



/**********************************************************************/
unsigned short apx15Gsof::getU16(unsigned char * * ppData)
/**********************************************************************/
// Used by the decoding routines to grab 2 bytes and pack them into
// a U16.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  unsigned short retValue ;

  retValue = *(*ppData)++ ;
  retValue <<= 8 ;
  retValue += *(*ppData)++ ;

  return retValue ;

} /* end of getU16() */


/**********************************************************************/
S16 apx15Gsof::getS16(unsigned char * * ppData)
/**********************************************************************/
// Used by the decoding routines to grab 2 bytes and pack them into
// a S16.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  S16 retValue ;

  retValue = *(*ppData)++ ;
  retValue <<= 8 ;
  retValue += *(*ppData)++ ;

  return retValue ;

} /* end of getS16() */


/**********************************************************************/
S32 apx15Gsof::getS32(unsigned char * * ppData)
/**********************************************************************/
// Used by the decoding routines to grab 4 bytes and pack them into
// a S32.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
{
  S32 retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 3 ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getS32() */



/***********************************************************************
 * The next section contains routines which are parsers for individual
 * GSOF records.  They are all passed a length (which is listed but
 * usually not used) and a pointer to the data bytes that make up the
 * record.
 ***********************************************************************
*/



/**********************************************************************/
void apx15Gsof::processEventMarker(int length, unsigned char *pData)
/**********************************************************************/
{
  unsigned char eventPort ;
  unsigned short eventWeekNumber ;
  double eventSeconds ;
  unsigned long eventNumber ;

  if (PRINT_DATA) {
      printf( "  GsofType:51 - EventMarker len:%d\n", length) ;
  }

  eventPort = *pData++ ;
  eventWeekNumber = getU16( &pData ) ;
  eventSeconds = getDouble( &pData ) ;
  eventNumber = getU32( &pData ) ;

  if (PRINT_DATA) {
      printf("  EvtMrk%u wk:%u sec:%.6f #:%lu\n",
      eventPort,
      eventWeekNumber,
      eventSeconds,
      eventNumber
      );
  }

} /* end of processEventMarker() */




/**********************************************************************/
void apx15Gsof::processINSNavigationInfo(int length, unsigned char *pData)
/**********************************************************************/
/* Parse the content of full navigation info for GSOF #49. pData is the pointer to the first byte of information in
* the message.
*/
{

  if (PRINT_DATA) {
    printf("  GsofType:49 INS Navigation info len:%d\n",
    length
  );
  }

  weekNum = getU16( &pData ) ;
  gpsTimeInMilliSec = getU32( &pData ) ;
  IMU_AlignmentStatus = *pData++ ;
  GPS_Quality = *pData++ ;
  latitude  = getDouble( &pData ) ;
  longitude = getDouble( &pData ) ;
  altitude  = getDouble( &pData ) ;

  northVelocity = getFloat( &pData ) ;
  eastVelocity  = getFloat( &pData ) ;
  downVelocity  = getFloat( &pData ) ;

  totalSpeed    = getFloat( &pData ) ;

  roll  = getDouble( &pData ) ;
  pitch = getDouble( &pData ) ;
  yaw   = getDouble( &pData ) ;

  trackAngle = getDouble( &pData ) ;

  angularRate[0] = getFloat( &pData ) ;
  angularRate[1] = getFloat( &pData ) ;
  angularRate[2] = getFloat( &pData ) ;

  acceleration[0] = getFloat( &pData ) ;
  acceleration[1] = getFloat( &pData ) ;
  acceleration[2] = getFloat( &pData ) ;

  if (PRINT_DATA) {
    printf(" GPS week              : %d                    \n"
      " GPS time (msec)       : %d                    \n"
      " IMU alignment status  : %d                    \n"
      " GPS Quality indicator : %d                    \n"
      " Latitude (degrees)    : %f                    \n"
      " Longitude (degrees)   : %f                    \n"
      " Altitude (m)          : %f                    \n"
      " North velocity (m/s)  : %f                    \n"
      " East velocity  (m/s)  : %f                    \n"
      " Down velocity  (m/s)  : %f                    \n"
      " Total speed (m/s)     : %f                    \n"
      " Roll     (degrees)    : %f                    \n"
      " Pitch    (degrees)    : %f                    \n"
      " Yaw      (degrees)    : %f                    \n"
      " Track angle (degrees) : %f                    \n"
      " Angular rate (X) (deg/sec)            : %f    \n"
      " Angular rate (Y) (deg/sec)            : %f    \n"
      " Angular rate (Z) (deg/sec)            : %f    \n"
      " Longitudinal accelaration (X) (m/s^2) : %f    \n"
      " Traverse acceleration (Y) (m/s^2)     : %f    \n"
      " Down acceleration (Z) (m/s^2)         : %f    \n",
      weekNum,
      gpsTimeInMilliSec,
      IMU_AlignmentStatus,
      GPS_Quality,
      latitude,
      longitude,
      altitude,
      northVelocity,
      eastVelocity,
      downVelocity,
      totalSpeed,
      roll,
      pitch,
      yaw,
      trackAngle,
      angularRate[0],
      angularRate[1],
      angularRate[2],
      acceleration[0],
      acceleration[1],
      acceleration[2]
      );

  }

  gsof31Hreceived=1;


} /* end of processINSNavigationInfo */



/**********************************************************************/
void apx15Gsof::processINSRMSInfo(int length, unsigned char *pData)
/**********************************************************************/
/* Parse the content of RMS info for GSOF #50. This message
* pData is the pointer to the first byte of information in the message.
*/
{
  if (PRINT_DATA) {
    printf("  GsofType:50 INS RMS info len:%d\n", length);
  }

  U16 weekNum = getU16( &pData ) ;
  U32 gpsTimeInMilliSec = getU32( &pData ) ;
  U8 IMU_AlignmentStatus = *pData++ ;
  U8 GPS_Quality = *pData++ ;

  northPositionRMS = getFloat( &pData ) ;
  eastPositionRMS  = getFloat( &pData ) ;
  downPositionRMS  = getFloat( &pData ) ;

  northVelocityRMS = getFloat( &pData ) ;
  eastVelocityRMS  = getFloat( &pData ) ;
  downVelocityRMS  = getFloat( &pData ) ;

  rollRMS     = getFloat( &pData ) ;
  pitchRMS    = getFloat( &pData ) ;
  yawRMS      = getFloat( &pData ) ;

  if (PRINT_DATA) {
    printf(" GPS week              : %d       \n"
      " GPS time (msec)       : %d       \n"
      " IMU alignment status  : %d       \n"
      " GPS Quality indicator : %d       \n"
      " North Position RMS (m): %f       \n"
      " East Position RMS (m) : %f       \n"
      " Down Position RMS (m) : %f       \n"
      " North velocity RMS (m/s)  : %f   \n"
      " East velocity  RMS (m/s)  : %f   \n"
      " Down velocity  RMS (m/s)  : %f   \n"
      " Roll RMS (degrees)        : %f   \n"
      " Pitch RMS (degrees)       : %f   \n"
      " Yaw RMS (degrees)     : %f   \n",
      weekNum,
      gpsTimeInMilliSec,
      IMU_AlignmentStatus,
      GPS_Quality,
      northPositionRMS,
      eastPositionRMS,
      downPositionRMS,
      northVelocityRMS,
      eastVelocityRMS,
      downVelocityRMS,
      rollRMS,
      pitchRMS,
      yawRMS
      );
  }

} /* end of processINSRMSInfo */

/**********************************************************************/
void apx15Gsof::processUTC(int length, unsigned char *pData)
/**********************************************************************/
/* Parse the content of UTC info for GSOF #16. This message
* pData is the pointer to the first byte of information in the message.
*/
{
  if (PRINT_DATA) {
    printf("  GsofType:x10h UTC info len:%d\n",
    length
    );
  }

  getU32( &pData ); // Time when packet is sent from the receiver, in GPS milliseconds of week
  getU16( &pData ); // Week number since the start of GPS time, already got it in #49.
  leapSecond = getS16(&pData);
  if(!leapSecondReceived) leapSecondReceived = true;
}

/***********************************************************************
 * End of the GSOF subtype parsers.
 **********************************************************************
*/



/**********************************************************************/
void apx15Gsof::processGsofData(void)
/**********************************************************************/
/* Called when a complete set of GSOF packets has been received.
* The data bytes collected are avialble in global gsofData and the
* number of those bytes is in gsofDataIndex.
*
* This routine just goes through the bytes and parses the sub-type
* records.  Each of those has a Type and a Length.  If the type is
* one of the special types we know about, we call the proper parser.
* Otherwise we just hex-dump the record.
*/
{
  int i ;
  int gsofType ;
  int gsofLength ;
  unsigned char * pData ;
  
  if (PRINT_DATA) {
    printf("\nGSOF Records\n");
  }

  pData = gsofData ;

  while (pData < gsofData + gsofDataIndex )
  {
    gsofType   = *pData++ ;
    gsofLength = *pData++ ;

    // If the type is one that we know about, then call the specific
    // parser for that type.
    switch (gsofType)
    {
      case 1 :
      case 2 :
      case 3 :
      case 4 :
      case 6 :
      case 7 :
      case 8 :
      case 9 :
      case 10 :
      case 11 :
      case 12 :
      case 13 :
      case 14 :
      case 15 :
      case 16 ://message type 10h, Current UTC time
        processUTC( gsofLength, pData);
        break;
      case 23 :
      case 25 :
      case 26 :
      case 27 :
      case 31 :
      case 33 :
      case 34 :
      case 35 :
      case 37 :
      case 40 :
      case 41 :
      case 44 :
      case 45 :
      case 46 :
      case 47 :
      case 52:
        break ;

      case 49 :
        processINSNavigationInfo( gsofLength, pData ) ;
        break ;

      case 50 :
        processINSRMSInfo( gsofLength, pData ) ;
        break ;

      case 51 :
        processEventMarker( gsofLength, pData ) ;
        break ;


      default:
      {
        if (PRINT_DATA) {
          // Not a type we know about.  Hex dump the bytes and move on.
          printf("  GsofType:%d  len:%d\n  ", gsofType, gsofLength);
          }

        U8 *p = pData ;
        for ( i = 0 ; i < gsofLength ; ++i )
        {
          if (PRINT_DATA) {
            printf( "%02X%s", *p++, i % 16 == 15 ? "\n  " : " ");
          }
        }
        // Terminate the last line if needed.
        if (gsofLength % 16 != 0) {
          if (PRINT_DATA)  {
            printf( "\n" ) ;
          }
        }
      }
    } // End of switch().

    // Step the data pointer past the full record.
    pData += gsofLength ;

    if (PRINT_DATA) {
      printf("\n");
    }
  }
  if (PRINT_DATA) {
    printf("\n");
  }

} /* end of processGsofData() */



/**********************************************************************/
void apx15Gsof::postGsofData(unsigned char * pData, int length)
/**********************************************************************/
// Called whenever we get a new Trimcomm GSOF packet (type 0x40).
// These all contain a portion (or all) of a complete GSOF packet.
// Each portion contains a Transmission Number, an incrementing value
// linking related portions.
// Each portion contains a Page Index, 0..N, which increments for each
// portion in the full GSOF packet.
// Each portion contains a Max Page Index, N, which is the same for all
// portions.
//
// Each portion's data is appended to the global buffer, gsofData[].
// The next available index in that buffer is always gsofDataIndex.
// When we receive a portion with Page Index == 0, that signals the
// beginning of a new GSOF packet and we restart the gsofDataIndex at
// zero.
//
// When we receive a portion where Page Index == Max Page Index, then
// we have received the complete GSOF packet and can decode it.
{
  int gsofTransmissionNumber ;
  int gsofPageIndex ;
  int gsofMaxPageIndex ;
  int i ;

  gsofTransmissionNumber = *pData++ ;
  gsofPageIndex = *pData++ ;
  gsofMaxPageIndex = *pData++ ;

  if (PRINT_DATA) {
    printf("  GSOF packet: Trans#:%d  Page:%d MaxPage:%d\n",
    gsofTransmissionNumber,
    gsofPageIndex,
    gsofMaxPageIndex
    );
  }

  // If this is the first portion, restart the buffering system.
  if (gsofPageIndex == 0)
    gsofDataIndex = 0 ;

  // Transfer the data bytes in this portion to the global buffer.
  for ( i = 3 ; i < length ; ++i )
    gsofData[ gsofDataIndex++ ] = *pData++ ;

  // If this is the last portion in a packet, process the whole packet.
  if (gsofPageIndex == gsofMaxPageIndex)
    processGsofData() ;

} /* end of postGsofData() */



unsigned char apx15Gsof::grabGsof(unsigned char * pData, int length)
{
  static int tcStx;
  static int tcStat;
  static int tcType;
  static int tcLength;
  static int tcCsum;
  static int tcEtx;
  static unsigned char tcData[256];

  int i;
    
  static unsigned char tcStxFlag = 0;
  static unsigned char counter1 = 0;
  static unsigned char headerFlag = 0;
  static unsigned char headerBuf[3];
  static unsigned char counter2 = 0;
  static unsigned char dataFlag = 0;
  static unsigned char counter3 = 0;
  static unsigned char endBuf[2];

  gsof31Hreceived=0;

  for (i = 0; i < length; i++)	{
        
    if (pData[i] == 0x02 && tcStxFlag == 0) {
      tcStx = pData[i];
      tcStxFlag = 1;
      counter1 = 0;
    }
    else {
      if (tcStxFlag == 1 && headerFlag == 0)  {
        headerBuf[counter1] = pData[i];
        counter1++;
        if (counter1 == 3)		{
          tcStat = headerBuf[0];
          tcType = headerBuf[1];
          tcLength = headerBuf[2];
          headerFlag = 1;
          counter2 = 0;
        }
      }
      else {

        if (headerFlag == 1 && dataFlag==0) {
          tcData[counter2] = pData[i];
          counter2++;
          if (counter2 == tcLength) {
              dataFlag = 1;
            counter3 = 0;
            }
        }
        else {
          if (dataFlag == 1) {
            endBuf[counter3] = pData[i];
            counter3++;
            if (counter3 == 2) {
              tcCsum = endBuf[0];
              tcEtx = endBuf[1];

              if (PRINT_DATA) {
                printf("STX:%02Xh  Stat:%02Xh  Type:%02Xh  "
                  "Len:%d  CS:%02Xh  ETX:%02Xh\n",
                  tcStx,
                  tcStat,
                  tcType,
                  tcLength,
                  tcCsum,
                  tcEtx
                  );
              }

              if (tcType == 0x40)
                postGsofData(tcData, tcLength);

              tcStxFlag = 0;
              headerFlag = 0;
              counter1 = 0;
              counter2 = 0;
              dataFlag = 0;
              counter3 = 0;
              
            }
          } //end if (dataFlag == 1) 
        }
      }
    }
  } // end of for (i = 0; i < length; i++)


  return(gsof31Hreceived);
}

} //namespace apx15



