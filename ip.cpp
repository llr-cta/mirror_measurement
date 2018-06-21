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

  if(argc!=1)
    {
      std::cerr << "Usage: " << program << " port" << '\n';
      exit(EXIT_FAILURE);
    }

  const char* port = *argv; 
  argv++,argc--;

  DataStream* ds = 0;

  try
    {
      ds = ESPProtocol::makeSerialDataStream(port);
      ESPProtocol esp(ds);
      esp.clearAllErrorCodes();
      
      while(1)
	{
	  uint8_t port_a_val;
	  uint8_t port_b_val;
	  uint8_t port_c_val;
	  esp.getDIOPortState(port_a_val, port_b_val, port_c_val);

	  for(unsigned ibit=0;ibit<8;ibit++)
	    std::cout << (((port_a_val>>(7-ibit))&0x01)?1:0);
	  std::cout << ' ';

	  for(unsigned ibit=0;ibit<8;ibit++)
	    std::cout << (((port_b_val>>(7-ibit))&0x01)?1:0);
	  std::cout << ' ';
	  
	  for(unsigned ibit=0;ibit<8;ibit++)
	    std::cout << (((port_c_val>>(7-ibit))&0x01)?1:0);
	  std::cout << '\n';

	  usleep(400000);
	}
    }
  catch(const CommunicationError& x)
    {
      x.print(std::cerr);
      std::cerr << strerror(x.errorNum()) << '\n';
    }

  delete ds;
}
