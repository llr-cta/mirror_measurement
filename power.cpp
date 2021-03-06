//-*-mode:c++; mode:font-lock;-*-

#include<time.h>
#include<unistd.h>

#include<DataStream.hpp>
#include<VSDataConverter.hpp>

#include"ESPProtocol.hpp"

using namespace VERITAS;
using namespace VTracking;
using namespace VMessaging;
using namespace MotionControl;

int main(int argc, char** argv)
{
  const char* program = *argv;
  argv++,argc--;

  if(argc!=3)
  {
    std::cerr << "Usage: " << program << " port axes [on/off]" << '\n';
    exit(EXIT_FAILURE);
  }

  const char* port = *argv;
  argv++,argc--;

  const char* axes = *argv;
  argv++,argc--;
  for(unsigned ichar=0; axes[ichar]; ichar++)
    if(axes[ichar]<'0' || axes[ichar]>'2')
    {
      std::cerr << "Unrecognised axis: " << axes[ichar] << " ; axes must be combination of 0, 1, and 2." << '\n';
      exit(EXIT_FAILURE);
    }

  const char* onoff = *argv;
  argv++,argc--;
  bool on = true;
  if(strcasecmp(onoff, "off")==0)on=false;
  else if(strcasecmp(onoff, "on")!=0)
  {
    std::cerr << "Final argument must be on or off." << '\n';
    exit(EXIT_FAILURE);
  }

  DataStream* ds = 0;

  try
  {
    ds = ESPProtocol::makeSerialDataStream(port);
    ESPProtocol esp(ds);
    esp.clearAllErrorCodes();
    bool axis_set[] = { false, false, false };
    for(unsigned ichar=0; axes[ichar]; ichar++)
    {
      ESPProtocol::IAxis iaxis = axes[ichar]-'0';
      if(axis_set[iaxis])continue;
      else if(on)esp.cmdMotorOn(iaxis);
      else esp.cmdMotorOff(iaxis);
      axis_set[iaxis] = true;
      usleep(100000);
    }
  }
  catch(const CommunicationError& x)
  {
    x.print(std::cerr);
    std::cerr << strerror(x.errorNum()) << '\n';
  }
  catch(const VMessaging::Exception& x)
  {
    x.print(std::cerr);
  }

  delete ds;
}
