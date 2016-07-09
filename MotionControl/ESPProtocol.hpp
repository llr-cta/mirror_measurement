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

#ifndef ESPPROTOCOL_HPP
#define ESPPROTOCOL_HPP

#include<string>
#include<list>
#include<cassert>

#include<VSDataConverter.hpp>
#include<DataStream.hpp>
#include<Exception.hpp>

namespace MotionControl
{

  class ESPProtocolException: public VMessaging::Exception
  {
  public:
    ESPProtocolException(const std::string& message="") throw():
      Exception("ESP Protocol Exception",message) { }
    virtual ~ESPProtocolException() throw();
  };

  class ESPProtocolInvalidParameter: public VMessaging::Exception
  {
  public:
    ESPProtocolInvalidParameter(const std::string& message="") throw():
      Exception("ESP Protocol Paremeter Invalid",message) { }
    virtual ~ESPProtocolInvalidParameter() throw();
  };
  
  class ESPProtocol
  {
  public:
    typedef unsigned               IAxis;
    typedef double                 Dist;
    typedef double                 Vel;
    typedef double                 Accel;
    typedef double                 Jerk;
    typedef unsigned               IDIO;
    typedef unsigned               TimeMS;
    typedef unsigned               TimeUS;
    typedef unsigned               IProgram;
    typedef unsigned               ILabel;

    static const IAxis noAxis = IAxis(-1);

    enum ProtocolVersion { PV_UNKNOWN, PV_ESP_300 };

    enum SerialSpeed { SS_9600, SS_19200, SS_38400, SS_57600, SS_115200 };

    enum HomeSearchMode { HSM_DEFAULT,
			  HSM_ZERO_POS, 
			  HSM_HOME_SWITCH_AND_INDEX_SIGNALS, 
			  HSM_HOME_SWITCH,
			  HSM_LIMIT_POS,
			  HSM_LIMIT_NEG,
			  HSM_LIMIT_POS_AND_INDEX_SIGNALS, 
			  HSM_LIMIT_NEG_AND_INDEX_SIGNALS };

    enum TrajectoryMode { TM_TRAPEZOIDAL,
			  TM_S_CURVE,
			  TM_JOG,
			  TM_SLAVE_TO_MASTER_DESIRED_POSITION,
			  TM_SLAVE_TO_MASTER_ACTUAL_POSITION,
			  TM_SLAVE_TO_MASTER_DESIRED_VELOCITY };

    enum Direction { D_POS, D_NEG };

    enum ErrorCode { EC_NONE                                           = 0,
		     EC_PCI_COMMUNICATION_TIMEOUT                      = 1,
		     EC_RESERVED_2                                     = 2,
		     EC_RESERVED_3                                     = 3,
		     EC_EMERGENCY_STOP_ACTIVATED                       = 4,
		     EC_COMMAND_DOES_NOT_EXIST                         = 6,
		     EC_PARAMETER_OUT_OF_RANGE                         = 7,
		     EC_CABLE_INTERLOCK_ERROR                          = 8,
		     EC_AXIS_NUMBER_OUT_OF_RANGE                       = 9,
		     EC_RESERVED_10                                    = 10,
		     EC_RESERVED_11                                    = 11,
		     EC_RESERVED_12                                    = 12,
		     EC_GROUP_NUMBER_MISSING                           = 13,
		     EC_GROUP_NUMBER_OUT_OF_RANGE                      = 14,
		     EC_GROUP_NUMBER_NOT_ASSIGNED                      = 15,
		     EC_GROUP_NUMBER_ALREADY_ASSIGNED                  = 16,
		     EC_GROUP_AXIS_OUT_OF_RANGE                        = 17,
		     EC_GROUP_AXIS_ALREADY_ASSIGNED                    = 18,
		     EC_GROUP_AXIS_DUPLICATED                          = 19,
		     EC_DATA_ACQUISITION_IS_BUSY                       = 20,
		     EC_DATA_ACQUISITION_SETUP_ERROR                   = 21,
		     EC_DATA_ACQUISITION_NOT_ENABLED                   = 22,
		     EC_SERVO_CYCLE_TICK_FAILURE                       = 23,
		     EC_RESERVED_24                                    = 24,
		     EC_DOWNLOAD_IN_PROGRESS                           = 25,
		     EC_STORED_PROGRAM_NOT_STARTED                     = 26,
		     EC_COMMAND_NOT_ALLOWED                            = 27,
		     EC_STORED_PROGRAM_FLASH_AREA_FULL                 = 28,
		     EC_GROUP_PARAMETER_MISSING                        = 29,
		     EC_GROUP_PARAMETER_OUT_OF_RANGE                   = 30,
		     EC_GROUP_MAXIMUM_VELOCITY_EXCEEDED                = 31,
		     EC_GROUP_MAXIMUM_ACCELERATION_EXCEEDED            = 32,
		     EC_GROUP_MAXIMUM_DECELERATION_EXCEEDED            = 33,
		     EC_GROUP_MOVE_NOT_ALLOWED_DURING_MOTION           = 34,
		     EC_PROGRAM_NOT_FOUND                              = 35,
		     EC_RESERVED_36                                    = 36,
		     EC_AXIS_NUMBER_MISSING                            = 37,
		     EC_COMMAND_PARAMETER_MISSING                      = 38,
		     EC_PROGRAM_LABEL_NOT_FOUND                        = 39,
		     EC_LAST_COMMAND_CANNOT_BE_REPEATED                = 40,
		     EC_MAXIMUM_NUMBER_OF_LABELS_PER_PROGRAM_EXCEEDED  = 41,

		     EC_AX_MOTOR_TYPE_NOT_DEFINED                      = 100,
		     EC_AX_PARAMETER_OUT_OF_RANGE                      = 101,
		     EC_AX_AMPLIFIER_FAULT_DETECTED                    = 102,
		     EC_AX_FOLLOWING_ERROR_THRESHOLD_EXCEEDED          = 103,
		     EC_AX_POSITIVE_HARDWARE_LIMIT_DETECTED            = 104,
		     EC_AX_NEGATIVE_HARDWARE_LIMIT_DETECTED            = 105,
		     EC_AX_POSITIVE_SOFTWARE_LIMIT_DETECTED            = 106,
		     EC_AX_NEGATIVE_SOFTWARE_LIMIT_DETECTED            = 107,
		     EC_AX_MOTOR_OR_STAGE_NOT_CONNECTED                = 108,
		     EC_AX_FEEDBACK_SIGNAL_FAULT_DETECTED              = 109,
		     EC_AX_MAXIMUM_VELOCITY_EXCEEDED                   = 110,
		     EC_AX_MAXIMUM_ACCELERATION_EXCEEDED               = 111,
		     EC_AX_RESERVED_X12                                = 112,
		     EC_AX_MOTOR_NOT_ENABLED                           = 113,
		     EC_AX_RESERVED_X14                                = 114,
		     EC_AX_MAXIMUM_JERK_EXCEEDED                       = 115,
		     EC_AX_MAXIMUM_DAC_OFFSET_EXCEEDED                 = 116,
		     EC_AX_ESP_CRITICAL_SETTINGS_ARE_PROTECTED         = 117,
		     EC_AX_ESP_STAGE_DEVICE_ERROR                      = 118,
		     EC_AX_ESP_STAGE_DATA_INVALID                      = 119,
		     EC_AX_HOME_ABORTED                                = 120,
		     EC_AX_MOTOR_CURRENT_NOT_DEFINED                   = 121,
		     EC_AX_UNIDRIVE_COMMUNICATIONS_ERROR               = 122,
		     EC_AX_UNIDRIVE_NOT_DETECTED                       = 123,
		     EC_AX_SPEED_OUT_OF_RANGE                          = 124,
		     EC_AX_INVALID_TRAJECTORY_MASTER_AXIS              = 125,
		     EC_AX_PARAMETER_CHANGE_NOT_ALLOWED                = 126,
		     EC_AX_INVALID_TRAJECTORY_MODE_FOR_HOMING          = 127,
		     EC_AX_INVALID_ENCODER_STEP_RATIO                  = 128,
		     EC_AX_DIGITAL_IO_INTERLOCK_DETECTED               = 129,
		     EC_AX_COMMAND_NOT_ALLOWED_DURING_HOMING           = 130,
		     EC_AX_COMMAND_NOT_ALLOWED_DUE_TO_GROUP_ASSIGNMENT = 131,
		     EC_AX_INVALID_TRAJECTORY_MODE_FOR_MOVING          = 132
    };

    typedef std::pair<ErrorCode,IAxis> ErrorListItem;
    typedef std::list<ErrorListItem> ErrorList;

    struct HardwareStatus
    {
      struct Axis
      {
	Axis(): limit_pos(), limit_neg(), amplifier_fault(),
		signal_home(), signal_index() { }
	
	bool limit_pos;
	bool limit_neg;
	bool amplifier_fault;
	bool signal_home;
	bool signal_index;
      };

      HardwareStatus(unsigned naxis=6, unsigned ndigital_ip=3):
	axis(naxis), 
	emergency_stop_unlatched_100pin(), emergency_stop_unlatched_aux_io(),
	emergency_stop_latched_100pin(), emergency_stop_latched_aux_io(),
	interlock_100pin(), digital_ip(ndigital_ip) { }

      std::vector<Axis> axis;
      bool emergency_stop_unlatched_100pin;
      bool emergency_stop_unlatched_aux_io;
      bool emergency_stop_latched_100pin;
      bool emergency_stop_latched_aux_io;
      bool interlock_100pin;
      std::vector<bool> digital_ip;
    };

    struct ControllerStatus
    {
      struct Axis
      {
	Axis(): in_motion() { }
	bool in_motion;
      };

      ControllerStatus(unsigned naxis=6):
	axis(naxis), power_on_at_least_one_axis() { }

      std::vector<Axis> axis;
      bool power_on_at_least_one_axis;
    };

    struct ControllerActivity
    {
      ControllerActivity():
	executing_at_least_one_program(), executing_wait(),
	executing_at_least_one_trajectory(), manual_jog_active(),
	local_mode_inactive() { }

      bool executing_at_least_one_program;
      bool executing_wait;
      bool executing_at_least_one_trajectory;
      bool manual_jog_active;
      bool local_mode_inactive;
    };

    ESPProtocol(VTracking::DataStream* ds);
    ESPProtocol(VTracking::DataStream& ds);
    ~ESPProtocol();

    static VTracking::DataStream* 
    makeSerialDataStream(const std::string& port, unsigned loud = 0,
			 SerialSpeed speed=SS_19200);
    
    static ProtocolVersion testDataStream(VTracking::DataStream& ds);

    // ERROR DETAILS ----------------------------------------------------------
    bool getErrorDetails(ErrorCode& errcode, IAxis& erriaxis,
			 unsigned& timestamp, std::string& message);
    bool getErrorCode(ErrorCode& errcode, IAxis& erriaxis);

    // CONTROLLER AND STAGE IDENTIFICATION ------------------------------------
    bool getStageModelAndSerial(IAxis iaxis, 
				std::string& model, std::string& serial);
    bool getControllerVersion(ProtocolVersion& protocol_version,
			      std::string& controller, 
			      unsigned& ver_major, unsigned& ver_minor);

    // EMERGENCY FUNCTIONS ----------------------------------------------------
    bool abortMotion();
    bool abortProgram();

    // CONTROLLER AND STAGE CONFIGURATION -------------------------------------
    bool getSerialSpeed(SerialSpeed& speed);
    bool getDACOffset(IAxis iaxis, double& volts);
    bool getControllerAddress(unsigned& address);
    bool getAvailableMemory(unsigned& bytes);

    bool resetController();
    bool setSerialSpeed(SerialSpeed speed);
    bool setDACOffset(IAxis iaxis, double volts);
    bool setControllerAddress(unsigned address);
    bool saveSettingsAndProgramsToNonVolatileMemory();
    
    // MOTION DEVICE PARAMETERS
    bool getLeftTravelLimit(IAxis iaxis, Dist& position);
    bool getRightTravelLimit(IAxis iaxis, Dist& position);

    bool setLeftTravelLimit(IAxis iaxis, Dist position);
    bool setRightTravelLimit(IAxis iaxis, Dist position);

    // TRAJECTORY DEFINITION --------------------------------------------------
    bool getJerk(IAxis iaxis, Jerk& jerk);
    bool getAcceleration(IAxis iaxis, Accel& accel);
    bool getDeceleration(IAxis iaxis, Accel& accel);
    bool getEmergencyStopDeceleration(IAxis iaxis, Accel& accel);
    bool getMaximumAcceleration(IAxis iaxis, Accel& accel);
    bool getSpeed(IAxis iaxis, Vel& speed);
    bool getMaximumSpeed(IAxis iaxis, Vel& speed);
    bool getJogHighSpeed(IAxis iaxis, Vel& speed);
    bool getJogLowSpeed(IAxis iaxis, Vel& speed);
    bool getHomeSearchHighSpeed(IAxis iaxis, Vel& speed);
    bool getHomeSearchLowSpeed(IAxis iaxis, Vel& speed);
    bool getHomeSearchMode(IAxis iaxis, HomeSearchMode& mode);
    bool getTrajectoryMode(IAxis iaxis, TrajectoryMode& mode);
    
    bool setJerk(IAxis iaxis, Jerk jerk);
    bool setAcceleration(IAxis iaxis, Accel accel);
    bool setDeceleration(IAxis iaxis, Accel accel);
    bool setEmergencyStopDeceleration(IAxis iaxis, Accel accel);
    bool setMaximumAcceleration(IAxis iaxis, Accel accel);
    bool setSpeed(IAxis iaxis, Vel speed);
    bool setMaximumSpeed(IAxis iaxis, Vel speed);
    bool setJogHighSpeed(IAxis iaxis, Vel speed);
    bool setJogLowSpeed(IAxis iaxis, Vel speed);
    bool setHomeSearchHighSpeed(IAxis iaxis, Vel speed);
    bool setHomeSearchLowSpeed(IAxis iaxis, Vel speed);
    bool setHomeSearchMode(IAxis iaxis, HomeSearchMode mode);
    bool setTrajectoryMode(IAxis iaxis, TrajectoryMode mode);

    // DIO CONFIGURATION ------------------------------------------------------
    bool getAssignDIOToExecuteStoredProgram(IDIO idio, IProgram iprog);
    bool getAssignDIOToInhibitMotion(IAxis iaxis, IDIO& idio, bool& level);
    bool getAssignDIOToNotifyMotionStatus(IAxis iaxis, IDIO& idio,bool& level);
    bool getAssignDIOForJogMode(IAxis iaxis, IDIO& idio_neg, IDIO& idio_pos);
    bool getEnableDIOToInhibitMotion(IAxis iaxis, bool& enable);
    bool getEnableDIOToNotifyMotionStatus(IAxis iaxis, bool& enable);
    bool getEnableDIOForJogMode(IAxis iaxis, bool& enable);
    bool getDIOPortDirection(bool& port_a_op, bool& port_b_op,bool& port_c_op);
    bool getDIOPortState(uint8_t& port_a_val, uint8_t& port_b_val, 
			 uint8_t& port_c_val);
    
    bool setAssignDIOToExecuteStoredProgram(IDIO idio, IProgram iprog);
    bool setAssignDIOToInhibitMotion(IAxis iaxis, IDIO idio, bool level);
    bool setAssignDIOToNotifyMotionStatus(IAxis iaxis, IDIO idio, bool level);
    bool setAssignDIOForJogMode(IAxis iaxis, IDIO idio_neg, IDIO idio_pos);
    bool setEnableDIOToInhibitMotion(IAxis iaxis, bool enable);
    bool setEnableDIOToNotifyMotionStatus(IAxis iaxis, bool enable);
    bool setEnableDIOForJogMode(IAxis iaxis, bool enable);
    bool setDIOPortDirection(bool port_a_op, bool port_b_op, bool port_c_op);
    bool setDIOPortState(uint8_t port_a_val, uint8_t port_b_val, 
			 uint8_t port_c_val);

    // CONTROLLER AND STAGE STATUS --------------------------------------------
    bool getHardwareStatus(HardwareStatus& status);
    bool getControllerStatus(ControllerStatus& status);
    bool getControllerActivity(ControllerActivity& activity);

    // MOTION -----------------------------------------------------------------
    bool getMotorOn(IAxis iaxis, bool& on);
    bool getMotorOff(IAxis iaxis, bool& off);
    bool getDesiredPosition(IAxis iaxis, Dist& position);
    bool getDesiredVelocity(IAxis iaxis, Vel& velocity);
    bool getActualPosition(IAxis iaxis, Dist& position);
    bool getActualVelocity(IAxis iaxis, Vel& velocity);
    bool getMotionDone(IAxis iaxis, bool& done);
    bool getMoveToHWTravelLimitDone(IAxis iaxis, bool& done);
    bool getMoveIndefinitelyDone(IAxis iaxis, bool& done);
    bool getMoveToNearestIndex(IAxis iaxis, bool& done);

    bool cmdMotorOn(IAxis iaxis);
    bool cmdMotorOff(IAxis iaxis);
    bool cmdMoveToHWTravelLimit(IAxis iaxis, Direction direction);
    bool cmdMoveIndefinitely(IAxis iaxis, Direction direction);
    bool cmdMoveToNearestIndex(IAxis iaxis, Direction direction);
    bool cmdMoveSearchForHome(IAxis iaxis, HomeSearchMode mode = HSM_DEFAULT);
    bool cmdMoveToAbsolutePosition(IAxis iaxis, Dist position);
    bool cmdMoveToRelativePosition(IAxis iaxis, Dist position);
    bool cmdStopMotion(IAxis iaxis);

    // PROGRAMMING ------------------------------------------------------------
    bool progBeginDownload(IProgram iprog);
    bool progFinishDownload();
    bool progDelete(IProgram iprog);
    //bool progList(IProgram iprog);
    bool progPurgeNonVolatileMemory();
    bool progGetAvailableMemory(IProgram iprog);
    bool progAutomaticExecutionOnPowerOn(IProgram iprog, unsigned ncycle=1);
    bool progDefineLabel(ILabel ilabel);
    bool progJumpToLabel(ILabel ilabel, unsigned ncycle=1);

    bool cmdExecuteProgram(IProgram iprog, unsigned ncycle=1);

    // WAIT FOR EVENTS --------------------------------------------------------
    bool waitForAbsolutePositionCrossing(IAxis iaxis, Dist position);
    bool waitForMotionDone(IAxis iaxis, TimeMS delay = 0);
    bool waitForTime(TimeMS delay);
    bool waitForDIOBitLo(IDIO idio); // Program mode only
    bool waitForDIOBitHi(IDIO idio); // Program mode only

    // SIMPLE IDIOMS ----------------------------------------------------------
    void clearAllErrorCodes();
    void getAllErrorCodes(ErrorList& list);    
    bool testForAndClearAllErrors();
    bool testForAndClearAllErrors(ErrorList& list);
    void pollForMotionDone(IAxis iaxis, TimeUS delay_us = 10000);
    bool cmdErrorFreeMoveToAbsolutePosition(IAxis iaxis, Dist position);
    bool cmdProtectedMoveToAbsolutePosition(IAxis iaxis, IDIO ibit,
					 Dist position, bool hit_value = true);

  protected:
    
    typedef std::vector<VTracking::datastring> Resp;
    
    void assertNResp(const Resp& resp, unsigned nresp)
    {
      VMessaging::RegisterThisFunction fnguard(__PRETTY_FUNCTION__);
      if(resp.size() != nresp)
	{
	  std::string msg("Response has ");
	  msg += VERITAS::VSDataConverter::toString(resp.size());
	  msg += ", expected ";
	  msg += VERITAS::VSDataConverter::toString(nresp);
	  throw ESPProtocolException(msg);
	}
    }

    void sendAndRecv(const VTracking::datastring& cmd, Resp& resp);
    
    inline VTracking::datastring cVoid(const std::string& cmd);
    inline VTracking::datastring cQuery(const std::string& cmd);
    template<typename T> inline VTracking::datastring 
    cSimple(const std::string& cmd, T x);
    template<typename T1, typename T2> inline VTracking::datastring 
    cSimplePair(const std::string& cmd, T1 x1, T2 x2);
    inline VTracking::datastring cHex(const std::string& cmd, unsigned x);
    inline VTracking::datastring 
    cHexPair(const std::string& cmd, unsigned x1, unsigned x2);

    inline VTracking::datastring aAxis(IAxis iaxis, const std::string& cmd);

    inline VTracking::datastring 
    cAxisVoid(IAxis iaxis, const std::string& cmd);
    inline VTracking::datastring 
    cAxisQuery(IAxis iaxis, const std::string& cmd);
    template<typename T> inline VTracking::datastring 
    cAxisSimple(IAxis iaxis, const std::string& cmd, T x);
    template<typename T1, typename T2> inline VTracking::datastring 
    cAxisSimplePair(IAxis iaxis, const std::string& cmd, T1 x1, T2 x2);
    inline VTracking::datastring 
    cAxisHex(IAxis iaxis, const std::string& cmd, unsigned x);
    inline VTracking::datastring 
    cAxisHexPair(IAxis iaxis, const std::string& cmd, unsigned x1,unsigned x2);

    template<typename T> inline bool 
    dSimple(const Resp& resp, T& x); 
    template<typename T1, typename T2> inline bool 
    dSimplePair(const Resp& resp, T1& x1, T2& x2); 
    template<typename T> inline bool 
    dVectorSimple(const Resp& resp, std::vector<T>& x, unsigned req_count = 0);
    inline bool dHex(const Resp& resp, unsigned& x);
    inline bool dHexPair(const Resp& resp, unsigned& x1, unsigned& x2);

  private:
    VTracking::DataStream& m_datastream;

    static const VTracking::datastring s_query;
    static const VTracking::datastring s_sEOM;
    static const VTracking::datastring s_rEOM;
  };

  VTracking::datastring ESPProtocol::
  cVoid(const std::string& cmd)
  {
    return cmd+s_sEOM;
  }

  VTracking::datastring ESPProtocol::
  cQuery(const std::string& cmd)
  {
    return cVoid(cmd+s_query);
  }

  template<typename T> inline VTracking::datastring ESPProtocol::
  cSimple(const std::string& cmd, T x)
  {
    return cVoid(cmd+VERITAS::VSDataConverter::toString(x,true));
  }

  template<typename T1, typename T2> inline VTracking::datastring ESPProtocol::
  cSimplePair(const std::string& cmd, T1 x1, T2 x2)
  {
    return cVoid(cmd
		 + VERITAS::VSDataConverter::toString(x1,true)
		 + std::string(",")
		 + VERITAS::VSDataConverter::toString(x2,true));
  }

  inline VTracking::datastring ESPProtocol::
  cHex(const std::string& cmd, unsigned x)
  {
    char buffer[40];
    sprintf(buffer,"%xH",x);
    return cVoid(cmd+std::string(buffer));
  }

  inline VTracking::datastring ESPProtocol::
  cHexPair(const std::string& cmd, unsigned x1, unsigned x2)
  {
    char buffer[80];
    sprintf(buffer,"%xH,%xH",x1,x2);
    return cVoid(cmd+std::string(buffer));
  }

  inline VTracking::datastring ESPProtocol::
  aAxis(IAxis iaxis, const std::string& cmd)
  {
    return VERITAS::VSDataConverter::toString(iaxis+1)+cmd;
  }

  VTracking::datastring ESPProtocol::
  cAxisVoid(IAxis iaxis, const std::string& cmd)
  {
    return cVoid(aAxis(iaxis,cmd));
  }

  VTracking::datastring ESPProtocol::
  cAxisQuery(IAxis iaxis, const std::string& cmd)
  {
    return aAxis(iaxis,cQuery(cmd));
  }

  template<typename T> inline VTracking::datastring ESPProtocol::
  cAxisSimple(IAxis iaxis, const std::string& cmd, T x)
  {
    return aAxis(iaxis,cSimple(cmd,x));
  }

  template<typename T1, typename T2> inline VTracking::datastring ESPProtocol::
  cAxisSimplePair(IAxis iaxis, const std::string& cmd, T1 x1, T2 x2)
  {
    return aAxis(iaxis, cSimplePair(cmd,x1,x2));
  }

  inline VTracking::datastring ESPProtocol::
  cAxisHex(IAxis iaxis, const std::string& cmd, unsigned x)
  {
    return aAxis(iaxis, cHex(cmd,x));
  }

  inline VTracking::datastring ESPProtocol::
  cAxisHexPair(IAxis iaxis, const std::string& cmd, unsigned x1, unsigned x2)
  {
    return aAxis(iaxis, cHexPair(cmd,x1,x2));
  }

  template<typename T> inline bool ESPProtocol::
  dSimple(const Resp& resp, T& x)
  {
    assertNResp(resp,1);
    return VERITAS::VSDataConverter::fromString(x,resp[0]);
  }

  template<typename T1, typename T2> inline bool ESPProtocol::
  dSimplePair(const Resp& resp, T1& x1, T2& x2)
  {
    assertNResp(resp,2);
    return ( VERITAS::VSDataConverter::fromString(x1,resp[0])
	     && VERITAS::VSDataConverter::fromString(x2,resp[1]) );
  }

  template<typename T> inline bool ESPProtocol::
  dVectorSimple(const Resp& resp, std::vector<T>& x, unsigned req_count)
  {
    if(req_count)assertNResp(resp,req_count);
    bool good = true;
    x.clear();
    for(unsigned iresp=0;iresp!=resp.size();iresp++)
      {
	T xi;
	good &= VERITAS::VSDataConverter::fromString(xi,resp[iresp]);
	x.push_back(xi);
      }
    return good;
  }

  inline bool ESPProtocol::dHex(const Resp& resp, unsigned& x)
  {
    assertNResp(resp,1);
    x = strtoul(resp[0].c_str(), 0, 16);
    return true;
  }

  inline bool ESPProtocol::
  dHexPair(const Resp& resp, unsigned& x1, unsigned& x2)
  {
    assertNResp(resp,1);
    x1 = strtoul(resp[0].c_str(), 0, 16);
    x2 = strtoul(resp[1].c_str(), 0, 16);
    return true;
  }

}

namespace VERITAS
{

#define __MAKEDATUMCONVERTER(T)						\
  template<> class VSDatumConverter<T>					\
  {									\
  public:								\
    static inline void							\
      toString(std::string& s, const T& x, bool low_precision=false);	\
    static inline bool fromString(T& x, const char* s);			\
    static inline std::string typeName();				\
  }

#define __MAKETOSTRING(T) \
  inline void VSDatumConverter<T>::				\
  toString(std::string& s, const T& x, bool low_precision)

#define __MAKEFROMSTRING(T) \
  inline bool VSDatumConverter<T>::fromString(T& x, const char* s)	\
  {									\
    assert(0);								\
  }

#define __MAKETYPENAME(T) \
  inline std::string VSDatumConverter<T>::typeName()	\
  {							\
    return std::string(#T);				\
  }
  
#define __MAKETOSTRINGCASE(c,x)			\
  case c::x: s=#x; break

  // --------------------------------------------------------------------------
  // MotionControl::ESPProtocol::ProtocolVersion ==============================
  // --------------------------------------------------------------------------

  __MAKEDATUMCONVERTER(MotionControl::ESPProtocol::ProtocolVersion);

  __MAKETOSTRING(MotionControl::ESPProtocol::ProtocolVersion)
  {
    switch(x)
      {
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,PV_UNKNOWN);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,PV_ESP_300);
      }
  }

  __MAKEFROMSTRING(MotionControl::ESPProtocol::ProtocolVersion)

  __MAKETYPENAME(MotionControl::ESPProtocol::ProtocolVersion)

  // --------------------------------------------------------------------------
  // MotionControl::ESPProtocol::SerialSpeed ==================================
  // --------------------------------------------------------------------------

  __MAKEDATUMCONVERTER(MotionControl::ESPProtocol::SerialSpeed);

  __MAKETOSTRING(MotionControl::ESPProtocol::SerialSpeed)
  {
    switch(x)
      {
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,SS_9600);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,SS_19200);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,SS_38400);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,SS_57600);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,SS_115200);
      }
  }

  __MAKEFROMSTRING(MotionControl::ESPProtocol::SerialSpeed)

  __MAKETYPENAME(MotionControl::ESPProtocol::SerialSpeed)

  // --------------------------------------------------------------------------
  // MotionControl::ESPProtocol::HomeSearchMode ===============================
  // --------------------------------------------------------------------------

  __MAKEDATUMCONVERTER(MotionControl::ESPProtocol::HomeSearchMode);

  __MAKETOSTRING(MotionControl::ESPProtocol::HomeSearchMode)
  {
    switch(x)
      {
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,HSM_DEFAULT);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,HSM_ZERO_POS);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   HSM_HOME_SWITCH_AND_INDEX_SIGNALS);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,HSM_HOME_SWITCH);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,HSM_LIMIT_POS);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,HSM_LIMIT_NEG);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   HSM_LIMIT_POS_AND_INDEX_SIGNALS);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   HSM_LIMIT_NEG_AND_INDEX_SIGNALS);
      }
  }

  __MAKEFROMSTRING(MotionControl::ESPProtocol::HomeSearchMode)

  __MAKETYPENAME(MotionControl::ESPProtocol::HomeSearchMode)

  // --------------------------------------------------------------------------
  // MotionControl::ESPProtocol::ProtocolVersion ==============================
  // --------------------------------------------------------------------------

  __MAKEDATUMCONVERTER(MotionControl::ESPProtocol::TrajectoryMode);

  __MAKETOSTRING(MotionControl::ESPProtocol::TrajectoryMode)
  {
    switch(x)
      {
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,TM_TRAPEZOIDAL);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,TM_S_CURVE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,TM_JOG);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   TM_SLAVE_TO_MASTER_DESIRED_POSITION);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   TM_SLAVE_TO_MASTER_ACTUAL_POSITION);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   TM_SLAVE_TO_MASTER_DESIRED_VELOCITY);
      }
  }

  __MAKEFROMSTRING(MotionControl::ESPProtocol::TrajectoryMode)

  __MAKETYPENAME(MotionControl::ESPProtocol::TrajectoryMode)

  // --------------------------------------------------------------------------
  // MotionControl::ESPProtocol::ProtocolVersion ==============================
  // --------------------------------------------------------------------------

  __MAKEDATUMCONVERTER(MotionControl::ESPProtocol::Direction);

  __MAKETOSTRING(MotionControl::ESPProtocol::Direction)
  {
    switch(x)
      {
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,D_POS);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,D_NEG);
      }
  }

  __MAKEFROMSTRING(MotionControl::ESPProtocol::Direction)

  __MAKETYPENAME(MotionControl::ESPProtocol::Direction)

  // --------------------------------------------------------------------------
  // MotionControl::ESPProtocol::ProtocolVersion ==============================
  // --------------------------------------------------------------------------

  __MAKEDATUMCONVERTER(MotionControl::ESPProtocol::ErrorCode);

  __MAKETOSTRING(MotionControl::ESPProtocol::ErrorCode)
  {
    switch(x)
      {
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_NONE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_PCI_COMMUNICATION_TIMEOUT);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_RESERVED_2);	
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_RESERVED_3);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_EMERGENCY_STOP_ACTIVATED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_COMMAND_DOES_NOT_EXIST);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_PARAMETER_OUT_OF_RANGE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_CABLE_INTERLOCK_ERROR);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AXIS_NUMBER_OUT_OF_RANGE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_RESERVED_10);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_RESERVED_11);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_RESERVED_12);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_NUMBER_MISSING);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_NUMBER_OUT_OF_RANGE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_NUMBER_NOT_ASSIGNED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_NUMBER_ALREADY_ASSIGNED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_AXIS_OUT_OF_RANGE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_AXIS_ALREADY_ASSIGNED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_AXIS_DUPLICATED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_DATA_ACQUISITION_IS_BUSY);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_DATA_ACQUISITION_SETUP_ERROR);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_DATA_ACQUISITION_NOT_ENABLED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_SERVO_CYCLE_TICK_FAILURE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_RESERVED_24);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_DOWNLOAD_IN_PROGRESS);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_STORED_PROGRAM_NOT_STARTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_COMMAND_NOT_ALLOWED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_STORED_PROGRAM_FLASH_AREA_FULL);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_PARAMETER_MISSING);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_PARAMETER_OUT_OF_RANGE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_MAXIMUM_VELOCITY_EXCEEDED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_MAXIMUM_ACCELERATION_EXCEEDED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_MAXIMUM_DECELERATION_EXCEEDED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_GROUP_MOVE_NOT_ALLOWED_DURING_MOTION);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_PROGRAM_NOT_FOUND);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_RESERVED_36);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AXIS_NUMBER_MISSING);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_COMMAND_PARAMETER_MISSING);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_PROGRAM_LABEL_NOT_FOUND);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_LAST_COMMAND_CANNOT_BE_REPEATED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_MAXIMUM_NUMBER_OF_LABELS_PER_PROGRAM_EXCEEDED);

	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_MOTOR_TYPE_NOT_DEFINED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_PARAMETER_OUT_OF_RANGE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_AMPLIFIER_FAULT_DETECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_FOLLOWING_ERROR_THRESHOLD_EXCEEDED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_POSITIVE_HARDWARE_LIMIT_DETECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_NEGATIVE_HARDWARE_LIMIT_DETECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_POSITIVE_SOFTWARE_LIMIT_DETECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_NEGATIVE_SOFTWARE_LIMIT_DETECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_MOTOR_OR_STAGE_NOT_CONNECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_FEEDBACK_SIGNAL_FAULT_DETECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_MAXIMUM_VELOCITY_EXCEEDED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_MAXIMUM_ACCELERATION_EXCEEDED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_RESERVED_X12);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_MOTOR_NOT_ENABLED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_RESERVED_X14);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_MAXIMUM_JERK_EXCEEDED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_MAXIMUM_DAC_OFFSET_EXCEEDED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_ESP_CRITICAL_SETTINGS_ARE_PROTECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_ESP_STAGE_DEVICE_ERROR);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_ESP_STAGE_DATA_INVALID);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,      
			   EC_AX_HOME_ABORTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_MOTOR_CURRENT_NOT_DEFINED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_UNIDRIVE_COMMUNICATIONS_ERROR);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_UNIDRIVE_NOT_DETECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_SPEED_OUT_OF_RANGE);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_INVALID_TRAJECTORY_MASTER_AXIS);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_PARAMETER_CHANGE_NOT_ALLOWED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_INVALID_TRAJECTORY_MODE_FOR_HOMING);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_INVALID_ENCODER_STEP_RATIO);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_DIGITAL_IO_INTERLOCK_DETECTED);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_COMMAND_NOT_ALLOWED_DURING_HOMING);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_COMMAND_NOT_ALLOWED_DUE_TO_GROUP_ASSIGNMENT);
	__MAKETOSTRINGCASE(MotionControl::ESPProtocol,
			   EC_AX_INVALID_TRAJECTORY_MODE_FOR_MOVING);
      }
  }

  __MAKEFROMSTRING(MotionControl::ESPProtocol::ErrorCode)

  __MAKETYPENAME(MotionControl::ESPProtocol::ErrorCode)

#undef __MAKEDATUMCONVERTER
#undef __MAKETOSTRING
#undef __MAKEFROMSTRING
#undef __MAKETYPENAME
#undef __MAKETOSTRINGCASE

}


#endif // not defined ESPPROTOCOL_HPP
