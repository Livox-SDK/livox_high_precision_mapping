#ifndef APX15_COMMS_H
#define APX15_COMMS_H

#include <stdint.h>
#include <string>

#include "serial/serial.h"
#include "apx15_gsof_parser.h"


namespace serial
{
class Serial;
}

namespace apx15
{


class Comms
{
public:
  Comms(serial::Serial* serial);
  ~Comms();

  int16_t receive(apx15Gsof* a);
 

private:
  serial::Serial* mySerial;

};

}  // namespace apx15

#endif  // APX15_COMMS_H

