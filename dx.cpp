//-*-mode:c++; mode:font-lock;-*-

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
      std::cerr << "Usage: " << program << " port axis distance" << '\n';
      exit(EXIT_FAILURE);
    }

  const char* port = *argv; 
  argv++,argc--;

  ESPProtocol::IAxis iaxis = 0;
  VSDataConverter::fromString(iaxis,*argv);
  argv++,argc--;

  ESPProtocol::Dist dx = 0;
  VSDataConverter::fromString(dx,*argv);
  argv++,argc--;

  DataStream* ds = 0;

  try
    {
      ds = ESPProtocol::makeSerialDataStream(port);
      ESPProtocol esp(ds);
      esp.clearAllErrorCodes();

      //esp.cmdMotorOn(iaxis);
      esp.cmdMoveToRelativePosition(iaxis,dx);
      esp.pollForMotionDone(iaxis);
      //usleep(400000);
      //esp.cmdMotorOff(iaxis);
    }
  catch(const CommunicationError& x)
    {
      x.print(std::cerr);
      std::cerr << strerror(x.errorNum()) << '\n';
    }

  delete ds;
}
