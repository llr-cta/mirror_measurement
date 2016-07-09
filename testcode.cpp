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

  if(argc!=1)
    {
      std::cerr << "Usage: " << program << " position" << '\n';
      exit(EXIT_FAILURE);
    }

  const char* port = "/dev/ttyS0";
  ESPProtocol::IAxis iaxis = 2;

  ESPProtocol::Dist z = 0;
  VSDataConverter::fromString(z,*argv);
  argv++,argc--;

  DataStream* ds = 0;

  try
    {
      ds = ESPProtocol::makeSerialDataStream(port);
      ESPProtocol esp(ds);
      esp.clearAllErrorCodes();

      esp.progBeginDownload(0);
      esp.cmdStopMotion(iaxis);
      esp.progFinishDownload();
 
      esp.setAssignDIOToExecuteStoredProgram(0, 0);

      esp.cmdMoveToAbsolutePosition(iaxis,z);
      esp.pollForMotionDone(iaxis);
      esp.getActualPosition(iaxis, z);
      std::cerr << z << std::endl;
    }
  catch(const CommunicationError& x)
    {
      x.print(std::cerr);
      std::cerr << strerror(x.errorNum()) << '\n';
    }

  delete ds;
}
