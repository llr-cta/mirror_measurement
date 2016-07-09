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

  if(argc!=4)
    {
      std::cerr << "Usage: " << program 
		<< " port axis small_step big_step" << '\n';
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

  ESPProtocol::Dist dx_big = 0;
  VSDataConverter::fromString(dx_big,*argv);
  argv++,argc--;

  DataStream* ds = 0;

  try
    {
      ds = ESPProtocol::makeSerialDataStream(port);
      ESPProtocol esp(ds);
      esp.clearAllErrorCodes();

      for(unsigned imainloop=0;imainloop<20000;imainloop++)
	{
	  bool hit = false;
	  uint8_t port_a_val;
	  uint8_t port_b_val;
	  uint8_t port_c_val;
	  esp.getDIOPortState(port_a_val, port_b_val, port_c_val);
	  hit = (port_a_val&0x01);

	  //unsigned ihitloop = 0;
	  while(!hit)
	    {
	      //std::cout << ++ihitloop << std::endl;
	      esp.cmdMoveToRelativePosition(iaxis,dx);
	      esp.pollForMotionDone(iaxis,5000);
	      esp.getDIOPortState(port_a_val, port_b_val, port_c_val);
	      hit = (port_a_val&0x01);
	    }

	  ESPProtocol::Dist position;
	  esp.getActualPosition(iaxis, position);

	  std::cerr << position << '\n';

	  esp.cmdMoveToRelativePosition(iaxis,dx_big);
	  esp.pollForMotionDone(iaxis,5000);
	}
    }
  catch(const CommunicationError& x)
    {
      x.print(std::cerr);
      std::cerr << strerror(x.errorNum()) << '\n';
    }

  delete ds;
}
