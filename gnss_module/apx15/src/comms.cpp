#include <ros/console.h>
#include "comms.h"

namespace apx15
{


Comms::Comms(serial::Serial* serial)
{
  mySerial=serial;

}

Comms::~Comms()
{

}


int16_t Comms::receive(apx15Gsof* myApx15 = NULL)
{
  unsigned char newPcaketFlag = 0;
  
  mySerial->waitReadable();
  int16_t availableBytes = mySerial->available();
  if (availableBytes > 1024) {
    ROS_WARN_STREAM("Serial read buffer is " << availableBytes << ", now flushing for catching up.");
    mySerial->flushInput();
  }

  uint8_t gsofBytes[1024];
  int16_t availableGot;

  if (availableBytes > 0) {
    availableGot=mySerial->read(gsofBytes, availableBytes);
    newPcaketFlag=myApx15->grabGsof((unsigned char*) &gsofBytes[0], availableGot);
  }

  return (newPcaketFlag);

}

}  // namespace apx15
