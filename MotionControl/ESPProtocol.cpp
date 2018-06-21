//-*-mode:c++; mode:font-lock;-*-

/**
 * \file ESPProtocol.hpp
 * \ingroup MotionControl
 * \brief Class implementing Newport ESP protocol language
 *
 * Original Author: Stephen Fegan
 * Original Date:   2007-09-24
 * $Author: sfegan $
 * $Date: 2008/04/15 17:22:36 $
 * $Revision: 1.14 $
 *
 **/

#include<string>
#include<sstream>
#include<unistd.h>

#include<VSDataConverter.hpp>
#include<Exception.hpp>
#include<Debug.hpp>

#include"ESPProtocol.hpp"

using namespace VERITAS;
using namespace VTracking;
using namespace VMessaging;
using namespace MotionControl;

const VTracking::datastring ESPProtocol::s_query("?");
const VTracking::datastring ESPProtocol::s_sEOM("\r");
const VTracking::datastring ESPProtocol::s_rEOM("\r\n");

#define CMD(x) #x

ESPProtocolException::
~ESPProtocolException() throw()
{
  // nothing to see here
}

ESPProtocolInvalidParameter::
~ESPProtocolInvalidParameter() throw()
{
  // nothing to see here
}

ESPProtocol::
ESPProtocol(VTracking::DataStream& ds):
  m_datastream(ds)
{
  // nothing to see here
}

ESPProtocol::
ESPProtocol(VTracking::DataStream* ds):
  m_datastream(*ds)
{
  assert(ds);
}

ESPProtocol::
~ESPProtocol()
{
  // nothing to see here
}

VTracking::DataStream* ESPProtocol::
makeSerialDataStream(const std::string& port, unsigned loud, SerialSpeed speed)
{
  tcflag_t baud = 0;
  switch(speed)
    {
    case SS_9600:   baud=B9600 ;   break;
    case SS_19200:  baud=B19200;   break;
    case SS_38400:  baud=B38400;   break;
    case SS_57600:  baud=B57600;   break;
    case SS_115200: baud=B115200;  break;
    };
  
  struct termios portconfig;
  memset(&portconfig,0,sizeof(portconfig));
  portconfig.c_iflag = IGNBRK;
  portconfig.c_oflag = 0;
  portconfig.c_cflag = ( baud | CS8 | CLOCAL | CREAD | CRTSCTS) & ~( PARENB );
  portconfig.c_lflag = ( NOFLSH ) & ~( ISIG );

  return new SerialPortDataStream(port,loud,0,500000,&portconfig);
}


ESPProtocol::ProtocolVersion ESPProtocol::
testDataStream(DataStream& ds)
{
  assert(0);
}

// ============================================================================
// ERROR DETAILS
// ============================================================================

bool ESPProtocol::
getErrorDetails(ErrorCode& errcode, IAxis& erriaxis,
 		unsigned& timestamp, std::string& message)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cVoid(CMD(TB)),resp);

  // Decode response ----------------------------------------------------------
  assertNResp(resp,3);
  unsigned code = 0;
  VSDataConverter::fromString(code,resp[0]);
  if(code<100)errcode=static_cast<ErrorCode>(code),erriaxis=noAxis;
  else errcode=static_cast<ErrorCode>(100+code%100),erriaxis=code/100-1;
  VSDataConverter::fromString(timestamp,resp[1]);
  message=resp[2];
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getErrorDetails() = " 
		    << VSDataConverter::toString(errcode) << ' '
		    << erriaxis << ' ' << timestamp << ' ' << message
		    << std::endl;  
  return true;  
}

bool ESPProtocol::
getErrorCode(ErrorCode& errcode, IAxis& erriaxis)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cVoid(CMD(TE)),resp);

  // Decode response ----------------------------------------------------------
  unsigned code = 0;
  dSimple(resp,code);
  if(code<100)errcode=static_cast<ErrorCode>(code),erriaxis=noAxis;
  else errcode=static_cast<ErrorCode>(100+code%100),erriaxis=code/100-1;
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getErrorCode() = " 
		    << VSDataConverter::toString(errcode) << ' '
		    << erriaxis << std::endl;  
  return true;
}

// ============================================================================
// CONTROLLER AND STAGE IDENTIFICATION
// ============================================================================

bool ESPProtocol::
getStageModelAndSerial(IAxis iaxis, std::string& model, std::string& serial)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(ID)),resp);

  // Decode response ----------------------------------------------------------
  assertNResp(resp,2);
  model = resp[0];
  serial = resp[1];
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getStageModelAndSerial(" 
		    << iaxis << ") = " 
		    << model << ' ' << serial << std::endl;  
  return true;
}

bool ESPProtocol::
getControllerVersion(ProtocolVersion& protocol_version,
		     std::string& controller, 
		     unsigned& ver_major, unsigned& ver_minor)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cQuery(CMD(VE)),resp);

  // Decode response ----------------------------------------------------------
  assertNResp(resp,1);
  std::istringstream str(resp[0]);
  str >> controller;
  
  if(controller == "ESP300")
    {
      protocol_version=PV_ESP_300;
      
      std::string version;
      str >> version;
      if(version != "Version")
	throw ESPProtocolInvalidParameter("Invalid controller version data: " 
					  + resp[0]);
      char c;
      str >> ver_major >> c >> ver_minor;
    }
  else 
    {
      protocol_version=PV_UNKNOWN;
      ver_major = 0;
      ver_minor = 0;
    }
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getControllerVersion() = "
		    << VSDataConverter::toString(protocol_version) << ' '
		    << controller << ' ' << ver_major << ' ' << ver_minor
		    << std::endl;  
  return true;
}

// ============================================================================
// EMERGENCY FUNCTIONS
// ============================================================================

bool ESPProtocol::abortMotion()
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cVoid(CMD(AB)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::abortMotion()" << std::endl;  
  return true;
}

bool ESPProtocol::abortProgram()
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cVoid(CMD(AP)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::abortProgram()" << std::endl;  
  return true;
}

// ============================================================================
// CONTROLLER AND STAGE CONFIGURATION
// ============================================================================

bool ESPProtocol::getSerialSpeed(SerialSpeed& speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cQuery(CMD(BR)),resp);

  // Decode response ----------------------------------------------------------
  unsigned ispeed = 0;
  dSimple(resp,ispeed);
  switch(ispeed)
    {
    case 9600:    speed=SS_9600;   break;
    case 19200:   speed=SS_19200;  break;
    case 38400:   speed=SS_38400;  break;
    case 57600:   speed=SS_57600;  break;
    case 115200:  speed=SS_115200; break;
    default:
      throw ESPProtocolInvalidParameter("Invalid baud rate: " 
					+ VSDataConverter::toString(ispeed));
      assert(0);
    };
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getSerialSpeed() = " 
		    << VSDataConverter::toString(speed)
		    << std::endl;  
  return true;
}

bool ESPProtocol::getDACOffset(IAxis iaxis, double& volts)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(DO)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,volts);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getDACOffset(" << iaxis << ") = " << volts
		    << std::endl;  
  return true;  
}

bool ESPProtocol::getControllerAddress(unsigned& address)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cQuery(CMD(SA)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,address);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getControllerAddress() = " << address
		    << std::endl;  
  return true;
}

bool ESPProtocol::getAvailableMemory(unsigned& bytes)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cVoid(CMD(XM)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,bytes);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getAvailableMemory() = "
		    << bytes << std::endl;
  return true;
}

bool ESPProtocol::resetController()
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cVoid(CMD(RS)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::resetController()" << std::endl;  
  return true;
}

bool ESPProtocol::setSerialSpeed(SerialSpeed speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  unsigned ispeed = 0;
  switch(speed)
    {
    case SS_9600:   ispeed=9600;   break;
    case SS_19200:  ispeed=19200;  break;
    case SS_38400:  ispeed=38400;  break;
    case SS_57600:  ispeed=57600;  break;
    case SS_115200: ispeed=115200; break;
    };
  m_datastream.sendData(cSimple(CMD(BR),ispeed));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setSerialSpeed(" << ispeed << ')'
		    << std::endl;  
  return true;  
}

bool ESPProtocol::setDACOffset(IAxis iaxis, double volts)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(DO),volts));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setDACOffset(" 
		    << iaxis << ',' << volts << ')'
		    << std::endl;  
  return true;    
}

bool ESPProtocol::setControllerAddress(unsigned address)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cSimple(CMD(SA),address));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setControllerAddress(" << address << ')'
		    << std::endl;  
  return true;      
}

bool ESPProtocol::saveSettingsAndProgramsToNonVolatileMemory()
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cVoid(CMD(SM)));
  if(m_datastream.loud()>=1)
    Debug::stream() 
      << "ESPProtocol::saveSettingsAndProgramsToNonVolatileMemory()"
      << std::endl;  
  return true;      
}

// ============================================================================
// MOTION DEVICE PARAMETERS
// ============================================================================

bool ESPProtocol::getLeftTravelLimit(IAxis iaxis, Dist& position)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(SL)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,position);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getLeftTravelLimit(" 
		    << iaxis << ") = " << position << std::endl;
  return true;
}

bool ESPProtocol::getRightTravelLimit(IAxis iaxis, Dist& position)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(SR)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,position);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getRightTravelLimit(" 
		    << iaxis << ") = " << position << std::endl;
  return true;
}

bool ESPProtocol::setLeftTravelLimit(IAxis iaxis, Dist position)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(SL),position));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setLeftTravelLimit(" 
		    << iaxis << ',' << position << ')'
		    << std::endl;  
  return true;    
}

bool ESPProtocol::setRightTravelLimit(IAxis iaxis, Dist position)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(SR),position));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setRightTravelLimit(" 
		    << iaxis << ',' << position << ')'
		    << std::endl;  
  return true;    
}

// ============================================================================
// TRAJECTORY DEFINITION
// ============================================================================

bool ESPProtocol::getJerk(IAxis iaxis, Jerk& jerk)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(JK)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,jerk);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getJerk(" 
		    << iaxis << ") = " << jerk << std::endl;
  return true;
}

bool ESPProtocol::getAcceleration(IAxis iaxis, Accel& accel)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(AC)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,accel);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getAcceleration(" 
		    << iaxis << ") = " << accel << std::endl;
  return true;
}

bool ESPProtocol::getDeceleration(IAxis iaxis, Accel& accel)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(AG)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,accel);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getDeceleration(" 
		    << iaxis << ") = " << accel << std::endl;
  return true;
}

bool ESPProtocol::getEmergencyStopDeceleration(IAxis iaxis, Accel& accel)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(AE)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,accel);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getEmergencyStopDeceleration(" 
		    << iaxis << ") = " << accel << std::endl;
  return true;
}

bool ESPProtocol::getMaximumAcceleration(IAxis iaxis, Accel& accel)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(AU)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,accel);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getMaximumAcceleration(" 
		    << iaxis << ") = " << accel << std::endl;
  return true;
}

bool ESPProtocol::getSpeed(IAxis iaxis, Vel& speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(VA)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,speed);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getSpeed(" 
		    << iaxis << ") = " << speed << std::endl;
  return true;
}

bool ESPProtocol::getMaximumSpeed(IAxis iaxis, Vel& speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(VU)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,speed);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getMaximumSpeed(" 
		    << iaxis << ") = " << speed << std::endl;
  return true;
}

bool ESPProtocol::getJogHighSpeed(IAxis iaxis, Vel& speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(JH)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,speed);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getJogHighSpeed(" 
		    << iaxis << ") = " << speed << std::endl;
  return true;
}

bool ESPProtocol::getJogLowSpeed(IAxis iaxis, Vel& speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(JW)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,speed);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getJogHighSpeed(" 
		    << iaxis << ") = " << speed << std::endl;
  return true;
}

bool ESPProtocol::getHomeSearchHighSpeed(IAxis iaxis, Vel& speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(OH)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,speed);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getHomeSearchHighSpeed(" 
		    << iaxis << ") = " << speed << std::endl;
  return true;
}

bool ESPProtocol::getHomeSearchLowSpeed(IAxis iaxis, Vel& speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(OL)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,speed);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getHomeSearchHighSpeed(" 
		    << iaxis << ") = " << speed << std::endl;
  return true;
}

bool ESPProtocol::getHomeSearchMode(IAxis iaxis, HomeSearchMode& mode)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(OM)),resp);

  // Decode response ----------------------------------------------------------
  unsigned imode = 0;
  dSimple(resp,imode);
  switch(imode)
    {
    case 0: mode=HSM_ZERO_POS;                       break;
    case 1: mode=HSM_HOME_SWITCH_AND_INDEX_SIGNALS;  break;
    case 2: mode=HSM_HOME_SWITCH;                    break;
    case 3: mode=HSM_LIMIT_POS;                      break;
    case 4: mode=HSM_LIMIT_NEG;                      break;
    case 5: mode=HSM_LIMIT_POS_AND_INDEX_SIGNALS;    break;
    case 6: mode=HSM_LIMIT_NEG_AND_INDEX_SIGNALS;    break;
    default:
      throw ESPProtocolInvalidParameter("Invalid home mode: " 
					+ VSDataConverter::toString(imode));
      assert(0);
    };
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getHomeSearchMode(" << iaxis << ") = " 
		    << VSDataConverter::toString(mode)
		    << std::endl;  
  return true;
}

bool ESPProtocol::getTrajectoryMode(IAxis iaxis, TrajectoryMode& mode)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(TJ)),resp);

  // Decode response ----------------------------------------------------------
  unsigned imode = 0;
  dSimple(resp,imode);
  switch(imode)
    {
    case 1: mode=TM_TRAPEZOIDAL;                      break;
    case 2: mode=TM_S_CURVE;                          break;
    case 3: mode=TM_JOG;                              break;
    case 4: mode=TM_SLAVE_TO_MASTER_DESIRED_POSITION; break;
    case 5: mode=TM_SLAVE_TO_MASTER_ACTUAL_POSITION;  break;
    case 6: mode=TM_SLAVE_TO_MASTER_DESIRED_VELOCITY; break;
    default:
      throw ESPProtocolInvalidParameter("Invalid trajectory mode: " 
					+ VSDataConverter::toString(imode));
      assert(0);
    };
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getTrajectoryMode(" << iaxis << ") = " 
		    << VSDataConverter::toString(mode) << std::endl;  
  return true;
}

bool ESPProtocol::setJerk(IAxis iaxis, Jerk jerk)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(JK),jerk));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setJerk(" 
		    << iaxis << ',' << jerk << ')'
		    << std::endl;  
  return true;    
}

bool ESPProtocol::setAcceleration(IAxis iaxis, Accel accel)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(AC),accel));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setAcceleration(" 
		    << iaxis << ',' << accel << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setDeceleration(IAxis iaxis, Accel accel)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(AG),accel));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setDeceleration(" 
		    << iaxis << ',' << accel << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setEmergencyStopDeceleration(IAxis iaxis, Accel accel)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(AE),accel));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setEmergencyStopDeceleration(" 
		    << iaxis << ',' << accel << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setMaximumAcceleration(IAxis iaxis, Accel accel)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(AU),accel));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setMaximumAcceleration(" 
		    << iaxis << ',' << accel << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setSpeed(IAxis iaxis, Vel speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(VA),speed));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setSpeed(" 
		    << iaxis << ',' << speed << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setMaximumSpeed(IAxis iaxis, Vel speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(VU),speed));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setMaximumSpeed(" 
		    << iaxis << ',' << speed << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setJogHighSpeed(IAxis iaxis, Vel speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(JH),speed));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setJogHighSpeed(" 
		    << iaxis << ',' << speed << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setJogLowSpeed(IAxis iaxis, Vel speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(JW),speed));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setJogLowSpeed(" 
		    << iaxis << ',' << speed << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setHomeSearchHighSpeed(IAxis iaxis, Vel speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(OH),speed));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setHomeSearchHighSpeed(" 
		    << iaxis << ',' << speed << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setHomeSearchLowSpeed(IAxis iaxis, Vel speed)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(OL),speed));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setHomeSearchLowSpeed(" 
		    << iaxis << ',' << speed << ')'
		    << std::endl;  
  return true;    
}
    
bool ESPProtocol::setHomeSearchMode(IAxis iaxis, HomeSearchMode mode)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  unsigned imode = 0;
  switch(mode)
    {
    case HSM_DEFAULT:
      throw ESPProtocolInvalidParameter("Invalid mode: HSM_DEFAULT");
      assert(0);
    case HSM_ZERO_POS:                      imode=0; break;
    case HSM_HOME_SWITCH_AND_INDEX_SIGNALS: imode=1; break;
    case HSM_HOME_SWITCH:                   imode=2; break;
    case HSM_LIMIT_POS:                     imode=3; break;
    case HSM_LIMIT_NEG:                     imode=4; break;
    case HSM_LIMIT_POS_AND_INDEX_SIGNALS:   imode=5; break;
    case HSM_LIMIT_NEG_AND_INDEX_SIGNALS:   imode=6; break;
    }
  m_datastream.sendData(cAxisSimple(iaxis,CMD(OM),imode));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setHomeSearchMode("
		    << iaxis << ',' << imode << ')'
		    << std::endl;  
  return true;      
}

bool ESPProtocol::setTrajectoryMode(IAxis iaxis, TrajectoryMode mode)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  unsigned imode = 0;
  switch(mode)
    {
    case TM_TRAPEZOIDAL:                      imode=1; break;
    case TM_S_CURVE:                          imode=2; break;
    case TM_JOG:                              imode=3; break;
    case TM_SLAVE_TO_MASTER_DESIRED_POSITION: imode=4; break;
    case TM_SLAVE_TO_MASTER_ACTUAL_POSITION:  imode=5; break;
    case TM_SLAVE_TO_MASTER_DESIRED_VELOCITY: imode=6; break;
    }
  m_datastream.sendData(cAxisSimple(iaxis,CMD(TJ),imode));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setTrajectoryMode("
		    << iaxis << ',' << imode << ')'
		    << std::endl;  
  return true;      
}

// ============================================================================
// DIO CONFIGURATION
// ============================================================================

bool ESPProtocol::
getAssignDIOToExecuteStoredProgram(IDIO idio, IProgram iprog)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(idio,CMD(BG)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,iprog);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getAssignDIOToExecuteStoredProgram(" 
		    << idio << ") = " << iprog << std::endl;
  return true;
}

bool ESPProtocol::
getAssignDIOToInhibitMotion(IAxis iaxis, IDIO& idio, bool& level)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(BK)),resp);

  // Decode response ----------------------------------------------------------
  dSimplePair(resp,idio,level);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getAssignDIOToInhibitMotion(" 
		    << iaxis << ") = " << idio << ' ' << level << std::endl;
  return true;
}

bool ESPProtocol::
getAssignDIOToNotifyMotionStatus(IAxis iaxis, IDIO& idio, bool& level)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(BM)),resp);

  // Decode response ----------------------------------------------------------
  dSimplePair(resp,idio,level);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getAssignDIOToNotifyMotionStatus(" 
		    << iaxis << ") = " << idio << ' ' << level << std::endl;
  return true;
}

bool  ESPProtocol::
getAssignDIOForJogMode(IAxis iaxis, IDIO& idio_neg, IDIO& idio_pos)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(BP)),resp);

  // Decode response ----------------------------------------------------------
  dSimplePair(resp,idio_neg,idio_pos);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getAssignDIOForJogMode(" 
		    << iaxis << ") = " << idio_neg << ' ' << idio_pos
		    << std::endl;
  return true;
}

bool ESPProtocol::getEnableDIOToInhibitMotion(IAxis iaxis, bool& enable)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(BL)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,enable);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getEnableDIOToInhibitMotion(" 
		    << iaxis << ") = " << enable << std::endl;
  return true;  
}

bool ESPProtocol::getEnableDIOToNotifyMotionStatus(IAxis iaxis, bool& enable)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(BN)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,enable);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getEnableDIOToNotifyMotionStatus(" 
		    << iaxis << ") = " << enable << std::endl;
  return true;  
}

bool ESPProtocol::getEnableDIOForJogMode(IAxis iaxis, bool& enable)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(BQ)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,enable);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getEnableDIOForJogMode(" 
		    << iaxis << ") = " << enable << std::endl;
  return true;  
}

bool ESPProtocol::getDIOPortDirection(bool& port_a_op, bool& port_b_op, 
				      bool& port_c_op)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cQuery(CMD(BO)),resp);

  // Decode response ----------------------------------------------------------
  unsigned reg;
  dHex(resp,reg);
  port_a_op = reg&0x01;
  port_b_op = reg&0x02;
  port_c_op = reg&0x04;
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getDIOPortDirection() = " 
		    << port_a_op << ' ' << port_b_op << ' ' << port_c_op
		    << std::endl;
  return true;  
}

bool ESPProtocol::getDIOPortState(uint8_t& port_a_val, uint8_t& port_b_val, 
				  uint8_t& port_c_val)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cQuery(CMD(SB)),resp);

  // Decode response ----------------------------------------------------------
  unsigned reg;
  dHex(resp,reg);
  port_a_val = (reg>>0)&0xFF;
  port_b_val = (reg>>8)&0xFF;
  port_c_val = (reg>>16)&0xFF;
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    {
      Debug::stream() << "ESPProtocol::getDIOPortState() = ";
      for(unsigned ibit=0;ibit<8;ibit++)
	Debug::stream() << (((port_a_val>>(7-ibit))&0x01)?1:0);
      Debug::stream() << ' ';
      for(unsigned ibit=0;ibit<8;ibit++)
	Debug::stream() << (((port_b_val>>(7-ibit))&0x01)?1:0);
      Debug::stream() << ' ';
      for(unsigned ibit=0;ibit<8;ibit++)
	Debug::stream() << (((port_c_val>>(7-ibit))&0x01)?1:0);
      Debug::stream() << std::endl;
    }

  return true;  
}

bool ESPProtocol::
setAssignDIOToExecuteStoredProgram(IDIO idio, IProgram iprog)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(idio,CMD(BG),iprog));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setAssignDIOToExecuteStoredProgram(" 
		    << idio << ',' << iprog << ')'
		    << std::endl;  
  return true;
}

bool ESPProtocol::
setAssignDIOToInhibitMotion(IAxis iaxis, IDIO idio, bool level)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimplePair(iaxis,CMD(BK),idio,level));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setAssignDIOToInhibitMotion(" 
		    << iaxis << ',' << idio << ',' << level << ')'
		    << std::endl;  
  return true;  
}

bool ESPProtocol::
setAssignDIOToNotifyMotionStatus(IAxis iaxis, IDIO idio, bool level)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimplePair(iaxis,CMD(BM),idio,level));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setAssignDIOToNotifyMotionStatus(" 
		    << iaxis << ',' << idio << ',' << level << ')'
		    << std::endl;  
  return true;    
}

bool ESPProtocol::
setAssignDIOForJogMode(IAxis iaxis, IDIO idio_neg, IDIO idio_pos)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimplePair(iaxis,CMD(BP),idio_neg,idio_pos));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setAssignDIOForJogMode(" 
		    << iaxis << ',' << idio_neg << ',' << idio_pos << ')'
		    << std::endl;  
  return true;    
}

bool ESPProtocol::setEnableDIOToInhibitMotion(IAxis iaxis, bool enable)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(BL),enable));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setEnableDIOToInhibitMotion(" 
		    << iaxis << ',' << enable << ')' << std::endl;  
  return true;    
}

bool ESPProtocol::setEnableDIOToNotifyMotionStatus(IAxis iaxis, bool enable)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(BN),enable));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setEnableDIOToNotifyMotionStatus(" 
		    << iaxis << ',' << enable << ')' << std::endl;  
  return true;    
}

bool ESPProtocol::setEnableDIOForJogMode(IAxis iaxis, bool enable)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(BQ),enable));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setEnableDIOForJogMode(" 
		    << iaxis << ',' << enable << ')' << std::endl;  
  return true;    
}

bool ESPProtocol::setDIOPortDirection(bool port_a_op, bool port_b_op,
				      bool port_c_op)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  unsigned reg = ( ((port_a_op)?0x01:0x00) 
		   | ((port_b_op)?0x02:0x00)
		   | ((port_c_op)?0x04:0x00) );
  m_datastream.sendData(cHex(CMD(BO),reg));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::setDIOPortDirection(" 
		    << port_a_op << ',' << port_b_op << ',' << port_c_op
		    << ')' << std::endl;  
  return true;
}

bool ESPProtocol::setDIOPortState(uint8_t port_a_val, uint8_t port_b_val, 
				  uint8_t port_c_val)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  unsigned reg = ( (unsigned(port_a_val)<<0)
		   | (unsigned(port_b_val)<<8)
		   | (unsigned(port_c_val)<<16) );
  m_datastream.sendData(cHex(CMD(SB),reg));
  if(m_datastream.loud()>=1)
    {
      Debug::stream() << "ESPProtocol::setDIOPortState(";
      for(unsigned ibit=0;ibit<8;ibit++)
	Debug::stream() << (((port_a_val>>(7-ibit))&0x01)?1:0);
      Debug::stream() << ',';
      for(unsigned ibit=0;ibit<8;ibit++)
	Debug::stream() << (((port_b_val>>(7-ibit))&0x01)?1:0);
      Debug::stream() << ',';
      for(unsigned ibit=0;ibit<8;ibit++)
	Debug::stream() << (((port_c_val>>(7-ibit))&0x01)?1:0);
      Debug::stream() << ')' << std::endl;  
    }
  return true;
}

// ============================================================================
// CONTROLLER AND STAGE CONFIGURATION
// ============================================================================

bool ESPProtocol::getHardwareStatus(HardwareStatus& status)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cVoid(CMD(PH)),resp);

  // Decode response ----------------------------------------------------------
  uint32_t reg1;
  uint32_t reg2;
  dHexPair(resp,reg1,reg2);

  status.axis.resize(6);
  for(unsigned iaxis=0;iaxis<status.axis.size();iaxis++)
    {
      status.axis[iaxis].limit_pos       = (reg1&(1<<(iaxis+0)));
      status.axis[iaxis].limit_neg       = (reg1&(1<<(iaxis+8)));
      status.axis[iaxis].amplifier_fault = (reg1&(1<<(iaxis+16)));
      status.axis[iaxis].signal_home     = (reg2&(1<<(iaxis+0)));
      status.axis[iaxis].signal_index    = (reg2&(1<<(iaxis+8)));
    }
  
  status.emergency_stop_unlatched_100pin = (reg1&(1<<27));
  status.emergency_stop_unlatched_aux_io = (reg1&(1<<28));
  status.emergency_stop_latched_100pin   = (reg1&(1<<29));
  status.emergency_stop_latched_aux_io   = (reg1&(1<<30));
  status.interlock_100pin                = (reg1&(1<<31));

  status.digital_ip.resize(3);
  for(unsigned idip=0;idip<status.digital_ip.size();idip++)
    {
      status.digital_ip[idip]            = (reg2&(1<<(idip+16)));
    }
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    {
      Debug::stream() << "ESPProtocol::getHardwareStatus() = ";
      for(unsigned iaxis=0;iaxis<status.axis.size();iaxis++)
	{
	  if(iaxis)Debug::stream() << ',';
	  Debug::stream() << '(' 
			  << (status.axis[iaxis].limit_pos?1:0) << ','
			  << (status.axis[iaxis].limit_neg?1:0) << ','
			  << (status.axis[iaxis].amplifier_fault?1:0) << ','
			  << (status.axis[iaxis].signal_home?1:0) << ','
			  << (status.axis[iaxis].signal_index?1:0) << ')';
	}

      Debug::stream() << ' '
		      << status.emergency_stop_unlatched_100pin << ' '
		      << status.emergency_stop_unlatched_aux_io << ' '
		      << status.emergency_stop_latched_100pin << ' '
		      << status.emergency_stop_latched_aux_io << ' '
		      << status.interlock_100pin << ' ';

      for(unsigned idip=0;idip<status.digital_ip.size();idip++)
	{
	  if(idip)Debug::stream() << ',';
	  Debug::stream() << (status.axis[idip].limit_pos?1:0);
	}
      
      Debug::stream() << std::endl;
    }
  return true;
}

bool ESPProtocol::getControllerStatus(ControllerStatus& status)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cVoid(CMD(TS)),resp);

  // Decode response ----------------------------------------------------------
  assertNResp(resp,1);
  uint8_t reg1 = resp[0][0];
  status.axis.resize(6);
  for(unsigned iaxis=0;iaxis<4;iaxis++)
    status.axis[iaxis].in_motion         = reg1&(1<<iaxis);
  status.power_on_at_least_one_axis      = reg1&(1<<4);
  if(resp[0].length()>1)
    {
      uint8_t reg2 = resp[0][1];
      status.axis[4].in_motion           = reg2&(1<<0);
      status.axis[5].in_motion           = reg2&(1<<1);
    }
  else
    {
      status.axis[4].in_motion           = false;
      status.axis[5].in_motion           = false;
    }
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    {
      Debug::stream() << "ESPProtocol::getControllerStatus() = ";
      for(unsigned iaxis=0;iaxis<status.axis.size();iaxis++)
	{
	  if(iaxis)Debug::stream() << ',';
	  Debug::stream() << status.axis[iaxis].in_motion;
	}

      Debug::stream() << ' ' << status.power_on_at_least_one_axis << std::endl;
    }
  return true;
}

bool ESPProtocol::getControllerActivity(ControllerActivity& activity)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cVoid(CMD(TX)),resp);

  // Decode response ----------------------------------------------------------
  assertNResp(resp,1);
  uint8_t reg1 = resp[0][0];
  activity.executing_at_least_one_program     = reg1&(1<<0);
  activity.executing_wait                     = reg1&(1<<1);
  activity.executing_at_least_one_trajectory  = reg1&(1<<4);
  activity.manual_jog_active                  = reg1&(1<<2);
  activity.local_mode_inactive                = reg1&(1<<3);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    {
      Debug::stream() << "ESPProtocol::getControllerActivity() = "
		      << activity.executing_at_least_one_program << ' '
		      << activity.executing_wait << ' '
		      << activity.executing_at_least_one_trajectory << ' '
		      << activity.manual_jog_active << ' '
		      << activity.local_mode_inactive << std::endl;
    }
  return true;
}

// ============================================================================
// MOTION
// ============================================================================

bool ESPProtocol::getMotorOn(IAxis iaxis, bool& on)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(MO)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,on);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getMotorOn(" << iaxis << ") = " << on
		    << std::endl;
  return true;
}

bool ESPProtocol::getMotorOff(IAxis iaxis, bool& off)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(MF)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,off);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getMotorOff(" << iaxis << ") = " << off
		    << std::endl;
  return true;
}

bool ESPProtocol::getDesiredPosition(IAxis iaxis, Dist& position)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(DP)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,position);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getDesiredPosition(" << iaxis << ") = "
		    << position << std::endl;
  return true;
}

bool ESPProtocol::getDesiredVelocity(IAxis iaxis, Vel& velocity)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(DV)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,velocity);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getDesiredVelocity(" << iaxis << ") = " 
		    << velocity << std::endl;
  return true;
}

bool ESPProtocol::getActualPosition(IAxis iaxis, Dist& position)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(TP)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,position);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getActualPosition(" << iaxis << ") = "
		    << position << std::endl;
  return true;
}

bool ESPProtocol::getActualVelocity(IAxis iaxis, Vel& velocity)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(TV)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,velocity);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getActualVelocity(" << iaxis << ") = "
		    << velocity << std::endl;
  return true;
}

bool ESPProtocol::getMotionDone(IAxis iaxis, bool& done)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(MD)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,done);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getMotionDone(" << iaxis << ") = " << done
		    << std::endl;
  return true;
  }

bool ESPProtocol::getMoveToHWTravelLimitDone(IAxis iaxis, bool& done)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(MT)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,done);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getMoveToHWTravelLimitDone(" << iaxis 
		    << ") = " << done << std::endl;
  return true;
}

bool ESPProtocol::getMoveIndefinitelyDone(IAxis iaxis, bool& done)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(MV)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,done);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getMoveIndefinitelyDone(" << iaxis 
		    << ") = " << done << std::endl;
  return true;
}

bool ESPProtocol::getMoveToNearestIndex(IAxis iaxis, bool& done)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  std::vector<VTracking::datastring> resp;
  sendAndRecv(cAxisQuery(iaxis,CMD(MZ)),resp);

  // Decode response ----------------------------------------------------------
  dSimple(resp,done);
  // --------------------------------------------------------------------------

  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::getMoveToNearestIndex(" << iaxis << ") = "
		    << done << std::endl;
  return true;
}

bool ESPProtocol::cmdMotorOn(IAxis iaxis)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisVoid(iaxis,CMD(MO)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdMotorOn(" << iaxis << ")" 
		    << std::endl;  
  return true;
}

bool ESPProtocol::cmdMotorOff(IAxis iaxis)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisVoid(iaxis,CMD(MF)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdMotorOff(" << iaxis << ")" 
		    << std::endl;  
  return true;
}

bool ESPProtocol::cmdMoveToHWTravelLimit(IAxis iaxis, Direction direction)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  const char* cmd = (direction==D_POS)?CMD(MT) "+":CMD(MT) "-";
  m_datastream.sendData(cAxisVoid(iaxis,cmd));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdMoveToHWTravelLimit(" << iaxis 
		    << ',' << cmd[2] << ')' << std::endl;  
  return true;
}

bool ESPProtocol::cmdMoveIndefinitely(IAxis iaxis, Direction direction)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  const char* cmd = (direction==D_POS)?CMD(MV) "+":CMD(MV) "-";
  m_datastream.sendData(cAxisVoid(iaxis,cmd));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdMoveIndefinitely(" << iaxis
		    << ',' << cmd[2] << ')' << std::endl;  
  return true;
}

bool ESPProtocol::cmdMoveToNearestIndex(IAxis iaxis, Direction direction)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  const char* cmd = (direction==D_POS)?CMD(MZ) "+":CMD(MZ) "-";
  m_datastream.sendData(cAxisVoid(iaxis,cmd));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdMoveToNearestIndex(" << iaxis
		    << ',' << cmd[2] << ')' << std::endl;  
  return true;
}

bool ESPProtocol::cmdMoveSearchForHome(IAxis iaxis, HomeSearchMode mode)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  const char* cmd = CMD(OR);
  if(mode==HSM_DEFAULT)
    m_datastream.sendData(cAxisVoid(iaxis,cmd));
  else
    {
      unsigned imode = 0;
      switch(mode)
	{
	case HSM_DEFAULT:                       assert(0); break;
	case HSM_ZERO_POS:                      imode=0; break;
	case HSM_HOME_SWITCH_AND_INDEX_SIGNALS: imode=1; break;
	case HSM_HOME_SWITCH:                   imode=2; break;
	case HSM_LIMIT_POS:                     imode=3; break;
	case HSM_LIMIT_NEG:                     imode=4; break;
	case HSM_LIMIT_POS_AND_INDEX_SIGNALS:   imode=5; break;
	case HSM_LIMIT_NEG_AND_INDEX_SIGNALS:   imode=6; break;
	}
      m_datastream.sendData(cAxisSimple(iaxis,cmd,imode));
    }
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdMoveSearchForHome(" << iaxis
		    << ',' << VSDataConverter::toString(mode) 
		    << ')' << std::endl;  
  return true;
}

bool ESPProtocol::cmdMoveToAbsolutePosition(IAxis iaxis, Dist position)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(PA),position));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdMoveToAbsolutePosition(" << iaxis
		    << ',' << position << ')' << std::endl;  
  return true;
}

bool ESPProtocol::cmdMoveToRelativePosition(IAxis iaxis, Dist position)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(PR),position));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdMoveToRelativePosition(" << iaxis
		    << ',' << position << ')' << std::endl;  
  return true;
}

bool ESPProtocol::cmdStopMotion(IAxis iaxis)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisVoid(iaxis,CMD(ST)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdStopMotion(" << iaxis << ')' 
		    << std::endl;  
  return true;
}

// ============================================================================
// PROGRAMMING
// ============================================================================

bool ESPProtocol::progBeginDownload(IProgram iprog)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisVoid(iprog,CMD(EP)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::progBeginDownload(" << iprog << ')' 
		    << std::endl;  
  return true;  
}

bool ESPProtocol::progFinishDownload()
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cVoid(CMD(QP)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::progFinishDownload()" << std::endl;  
  return true;    
}

bool ESPProtocol::progDelete(IProgram iprog)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisVoid(iprog,CMD(XX)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::progDelete(" << iprog << ')' 
		    << std::endl;  
  return true;    
}

#if 0
bool ESPProtocol::progList(IProgram iprog)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisVoid(iprog,CMD(LP)));
  
}
#endif

bool ESPProtocol::progPurgeNonVolatileMemory()
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cVoid(CMD(0XX)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::progPurgeNonVolatileMemory()"
		    << std::endl;  
  return true;    
}

bool ESPProtocol::
progAutomaticExecutionOnPowerOn(IProgram iprog, unsigned ncycle)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iprog,CMD(EO),ncycle));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::progAutomaticExecutionOnPowerOn("
		    << iprog << ',' << ncycle << ')' << std::endl;  
  return true;  
}

bool ESPProtocol::progDefineLabel(ILabel ilabel)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisVoid(ilabel,CMD(DL)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::progDefineLabel(" << ilabel << ')' 
		    << std::endl;  
  return true;
}

bool ESPProtocol::progJumpToLabel(ILabel ilabel, unsigned ncycle)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(ilabel,CMD(JL),ncycle));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::progJumpToLabel(" << ilabel
		    << ',' << ncycle << ')' << std::endl;  
  return true;
}

bool ESPProtocol::cmdExecuteProgram(IProgram iprog, unsigned ncycle)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  //if(ncycle==0)return false;
  m_datastream.sendData(cAxisSimple(iprog,CMD(EX),ncycle));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::cmdExecuteProgram(" << iprog
		    << ',' << ncycle << ')' << std::endl;  
  return true;
}

// ============================================================================
// WAIT FOR EVENTS
// ============================================================================

bool ESPProtocol::waitForAbsolutePositionCrossing(IAxis iaxis, Dist position)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisSimple(iaxis,CMD(WP),position));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::waitForAbsolutePositionCrossing(" 
		    << iaxis << ',' << position << ')' << std::endl;  
  return true;
}

bool ESPProtocol::waitForMotionDone(IAxis iaxis, TimeMS delay)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  if(delay==0)
    m_datastream.sendData(cAxisVoid(iaxis,CMD(WS)));
  else
    m_datastream.sendData(cAxisSimple(iaxis,CMD(WS),delay));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::waitForMotionDone(" 
		    << iaxis << ',' << delay << ')' << std::endl;  
  return true;
}

bool ESPProtocol::waitForTime(TimeMS delay)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cSimple(CMD(WT),delay));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::waitForTime(" << delay << ')'
		    << std::endl;  
  return true;
}

bool ESPProtocol::waitForDIOBitLo(IDIO idio)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisVoid(idio,CMD(UL)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::waitForDIOBitLo(" << idio << ')'
		    << std::endl;  
  return true;
}

bool ESPProtocol::waitForDIOBitHi(IDIO idio)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cAxisVoid(idio,CMD(UH)));
  if(m_datastream.loud()>=1)
    Debug::stream() << "ESPProtocol::waitForDIOBitHi(" << idio << ')'
		    << std::endl;  
  return true;
}

// ============================================================================
// SIMPLE IDIOMS
// ============================================================================
    
void ESPProtocol::clearAllErrorCodes()
{
  ESPProtocol::ErrorCode errorcode;
  ESPProtocol::IAxis erroraxis;      
  getErrorCode(errorcode,erroraxis);
  while(errorcode!=ESPProtocol::EC_NONE)getErrorCode(errorcode,erroraxis);
}

void ESPProtocol::getAllErrorCodes(ErrorList& list)
{
  list.clear();
  ESPProtocol::ErrorCode errorcode;
  ESPProtocol::IAxis erroraxis;      
  getErrorCode(errorcode,erroraxis);
  while(errorcode!=ESPProtocol::EC_NONE)
    {
      list.push_back(ErrorListItem(errorcode,erroraxis));
      getErrorCode(errorcode,erroraxis);
    }
}

bool ESPProtocol::testForAndClearAllErrors()
{
  ErrorList list;
  getAllErrorCodes(list);
  return !list.empty();
}

bool ESPProtocol::testForAndClearAllErrors(ErrorList& list)
{
  getAllErrorCodes(list);
  return !list.empty();
}

void ESPProtocol::pollForMotionDone(IAxis iaxis, TimeUS delay_us)
{
  bool done = false; 
  getMotionDone(iaxis,done);
  while(!done)
    {
      if(delay_us)usleep(delay_us);
      getMotionDone(iaxis,done);
    }
}

bool ESPProtocol::
cmdErrorFreeMoveToAbsolutePosition(IAxis iaxis, Dist position)
{
  clearAllErrorCodes();
  cmdMoveToAbsolutePosition(iaxis, position);
  pollForMotionDone(iaxis);
  return !testForAndClearAllErrors();
}

bool ESPProtocol::
cmdProtectedMoveToAbsolutePosition(IAxis iaxis, IDIO ibit,
				   Dist position, bool hit_value)
{
  clearAllErrorCodes();
  setAssignDIOToInhibitMotion(iaxis, ibit, hit_value);
  setEnableDIOToInhibitMotion(iaxis, true);
  cmdMoveToAbsolutePosition(iaxis, position);
  pollForMotionDone(iaxis);
  bool error = testForAndClearAllErrors();
  setEnableDIOToInhibitMotion(iaxis, false);
  return !error;
}

// ============================================================================
// ============================================================================
// ============================================================================
// 
// PROTECTED FUNCTIONS
// 
// ============================================================================
// ============================================================================
// ============================================================================

void ESPProtocol::sendAndRecv(const VTracking::datastring& cmd,
			      std::vector<VTracking::datastring>& resp)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
  m_datastream.sendData(cmd);
  datastring data = m_datastream.recvToEOM(s_rEOM);

  if(m_datastream.loud()>=2)
    {
      Debug::stream() << "ESPProtocol::sendAndRecv(): ";
      DataStream::printAsAscii(Debug::stream(), data);
      Debug::stream() << std::endl;
    }

  resp.clear();

  datastring::size_type idata = 0;
  datastring::size_type ndata = data.size();
  while(1)
    {
      while((idata != ndata)&&(data[idata] != s_rEOM[0])
	    &&(isspace(data[idata])))idata++;

      datastring::size_type ifind = idata;
      datastring::size_type inows = ifind-1;
      while((ifind != ndata)&&(data[ifind] != s_rEOM[0])&&(data[ifind] != ','))
	{
	  if(!isspace(data[ifind]))inows=ifind;
	  ifind++;
	}

      assert(ifind != ndata);
      resp.push_back(data.substr(idata,inows-idata+1));
      if(data[ifind] == s_rEOM[0])break;
      else idata=ifind+1;
    }
}

// ============================================================================
// ============================================================================
// ============================================================================
//
// VSDATACONVERTERs
//
// ============================================================================
// ============================================================================
// ============================================================================
