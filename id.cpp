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

  const char* port = "/dev/ttyS0";
  if(argc)port=*argv;
  argv++,argc--;

  DataStream* ds = 0;

  try
    {
      ds = ESPProtocol::makeSerialDataStream(port);
      ESPProtocol esp(ds);
      esp.clearAllErrorCodes();

      ESPProtocol::ProtocolVersion pv;
      std::string cs;
      unsigned vmaj;
      unsigned vmin;
      esp.getControllerVersion(pv,cs,vmaj,vmin);

      if(esp.testForAndClearAllErrors())
	{
	  std::cerr << "ESP controller has error condition" << std::endl;
	  exit(EXIT_FAILURE);
	}

      std::cout << "Controller on port " << port << ": "
		<< cs << ' ' << vmaj << '.' << vmin << '\n';

      for(ESPProtocol::IAxis iaxis=0;iaxis<3;iaxis++)
	{
	  std::string model;
	  std::string serial;
	  esp.getStageModelAndSerial(iaxis, model, serial);

	  if(esp.testForAndClearAllErrors())
	    {
	      std::cout << "Axis " << iaxis << ": no stage" << '\n';
	      continue;
	    }

	  std::cout << '\n';
	  std::cout << "Axis " << iaxis << ": " 
		    << model << ' ' << serial << '\n';

	  ESPProtocol::TrajectoryMode t_mode;
	  esp.getTrajectoryMode(iaxis, t_mode);
	  std::cout << "  Trajectory mode: " 
		    << VSDataConverter::toString(t_mode) << '\n';

	  ESPProtocol::Jerk jerk;
	  esp.getJerk(iaxis, jerk);
	  std::cout << "  Jerk:            " << jerk << '\n';
	  
	  ESPProtocol::Accel accel;
	  esp.getAcceleration(iaxis, accel);
	  std::cout << "  Acceleration:    " << accel << '\n';

	  esp.getDeceleration(iaxis, accel);
	  std::cout << "  Deceleration:    " << accel << '\n';

	  esp.getEmergencyStopDeceleration(iaxis, accel);
	  std::cout << "  EStop decel:     " << accel << '\n';

	  esp.getMaximumAcceleration(iaxis, accel);
	  std::cout << "  Max decel:       " << accel << '\n';

	  ESPProtocol::Vel speed;
	  esp.getSpeed(iaxis, speed);
	  std::cout << "  Speed:           " << speed << '\n';

	  esp.getMaximumSpeed(iaxis, speed);
	  std::cout << "  Max speed:       " << speed << '\n';

	  esp.getJogHighSpeed(iaxis, speed);
	  std::cout << "  Jog high speed:  " << speed << '\n';

	  esp.getJogLowSpeed(iaxis, speed);
	  std::cout << "  Jog low speed:   " << speed << '\n';

	  esp.getHomeSearchHighSpeed(iaxis, speed);
	  std::cout << "  HS high speed:   " << speed << '\n';

	  esp.getHomeSearchLowSpeed(iaxis, speed);
	  std::cout << "  HS low speed:    " << speed << '\n';

	  ESPProtocol::HomeSearchMode hs_mode;
	  esp.getHomeSearchMode(iaxis, hs_mode);
	  std::cout << "  HS mode:         " 
		    << VSDataConverter::toString(hs_mode) << '\n';

	  ESPProtocol::Dist pos;
	  esp.getLeftTravelLimit(iaxis, pos);
	  std::cout << "  Left limit:      " << pos << '\n';

	  esp.getRightTravelLimit(iaxis, pos);
	  std::cout << "  Right limit:     " << pos << '\n';
	}
    }
  catch(const CommunicationError& x)
    {
      x.print(std::cerr);
      std::cerr << strerror(x.errorNum()) << '\n';
    }

  delete ds;
}
