//-*-mode:c++; mode:font-lock;-*-

#include<cmath>
#include<map>

#include<time.h>
#include<unistd.h>

#include<DataStream.hpp>
#include<VSOptions.hpp>
#include<VSSimpleStat.hpp>
#include<VSDataConverter.hpp>
#include<Exception.hpp>
#include<Debug.hpp>

#include"ESPProtocol.hpp"

using namespace VERITAS;
using namespace VTracking;
using namespace VMessaging;
using namespace MotionControl;

#define SQR(x) ((x)*(x))

struct Abort {};

struct Config 
{
  typedef ESPProtocol::IAxis IAxis;
  typedef ESPProtocol::IDIO IDIO;
  typedef ESPProtocol::Dist Dist;

  Config(): 
    dev_probe("/dev/ttyUSBS0"), ax_probe_x(0), ax_probe_y(1), ax_probe_z(2),
    ibit_hit(0), ibit_alive(1), bit_hit_value(true), bit_alive_value(true),
    dev_mirror("/dev/ttyUSBS1"), ax_mirror_x(0), ax_mirror_z(1), ax_mirror_t(2),
    limit_abs_z_bwd(0.0), limit_abs_z_fwd(9999.0), 
    limit_rel_z(0.25), limit_rel_z_zero_est_mult(4),
    seek_step_big(0.1), seek_nstep_big_back(10),
    seek_step_small(0.001), 
    seek_nstep_small_back_first(30), seek_nstep_small_back(10)
  { }

  std::string dev_probe;    
  IAxis       ax_probe_x;
  IAxis       ax_probe_y;
  IAxis       ax_probe_z;
  IDIO        ibit_hit;
  IDIO        ibit_alive;
  bool        bit_hit_value;
  bool        bit_alive_value;

  std::string dev_mirror;    
  IAxis       ax_mirror_x;
  IAxis       ax_mirror_z;
  IAxis       ax_mirror_t;

  Dist        limit_abs_z_bwd;
  Dist        limit_abs_z_fwd;
  Dist        limit_rel_z;
  unsigned    limit_rel_z_zero_est_mult;

  Dist        seek_step_big;
  unsigned    seek_nstep_big_back_first;
  unsigned    seek_nstep_big_back;
  Dist        seek_step_small;
  unsigned    seek_nstep_small_back_first;
  unsigned    seek_nstep_small_back;

  void configure(VSOptions& opt);
};

void Config::configure(VSOptions& opt)
{
  opt.findWithValue("limit_abs_z_bwd",limit_abs_z_bwd,
		    "Set the absolute limit on motion in the negative-z "
		    "direction. This limit can be used to stop the probe "
		    "mounting from impacting the actuator. It is also used to "
		    "slew over obstacles on the mirror. Specify limit in mm.");
  opt.findWithValue("limit_abs_z_fwd",limit_abs_z_fwd,
		    "Set the absolute limit on motion in the positive-z "
		    "direction. Limit should be given in mm.");
  opt.findWithValue("limit_rel_z",limit_rel_z,
		    "Set the limit on motion in the positive-z direction past "
		    "the estimated mirror surface when good estimates are "
		    "available based on previous data. This limit protects "
		    "the probe from hitting the edge of the mirror or holes "
		    "in its surface. Specify limit in mm.");
  opt.findWithValue("limit_rel_z_zero_est_multiplier", 
		    limit_rel_z_zero_est_mult,
		    "Multiplier to apply to relative z limit when no data are "
		    "available to base estimate on.");

  opt.findWithValue("seek_step_big",seek_step_big,
		    "Size of large forward step when searching for mirror "
		    "surface (in mm).");
  opt.findWithValue("seek_nstep_big_back",seek_nstep_big_back,
		    "Number of large steps to make backwards when moving from "
		    "place to place on the mirror.");

  opt.findWithValue("seek_step_small",seek_step_small,
		    "Size of small step used when finding mirror surface.");
  opt.findWithValue("seek_nstep_small_back_first",
		    seek_nstep_small_back_first,
		    "Number of small steps away from estimated mirror that "
		    "probe should be positioned after first slewing to new "
		    "mirror point.");
  opt.findWithValue("seek_nstep_small_back",seek_nstep_small_back,
		    "Number of small steps to make away from mirror when "
		    "making muliple samples.");

  seek_step_big = std::min(seek_step_big, 0.5);

  assert(limit_abs_z_fwd>=limit_abs_z_bwd);
  assert(limit_rel_z>0);
  assert(limit_rel_z_zero_est_mult>0);
  assert(seek_step_big>0.0);
  assert(seek_nstep_big_back>0);
  assert(seek_step_small>0.0);
  assert(seek_nstep_small_back_first>0);
  assert(seek_nstep_small_back>0);
}

void usage(const std::string& progname, const VSOptions& options,
           std::ostream& stream)
{
  stream << "Usage: " << progname  << " [options] x/y [x/y...]\n\n"
	 << "Where x/y are the coordinates of the points to test, in mm\n";
  stream << "Options:" << std::endl;
  options.printUsage(stream);
}

bool isProbeTriggered(ESPProtocol* esp, const Config& C)
{
  uint8_t port_a_val;
  uint8_t port_b_val;
  uint8_t port_c_val;
  esp->getDIOPortState(port_a_val, port_b_val, port_c_val);
  
  if((port_a_val&(0x01<<C.ibit_alive)?true:false)!=C.bit_alive_value)
    {
      std::cerr << "Touch sensor probe not seated or powered off" << std::endl;
      exit(EXIT_FAILURE);
    }

  return port_a_val&0x01;
}

typedef ESPProtocol::Dist           Dist;
typedef std::pair<Dist,Dist>        Coord;
typedef triple<Dist, Dist, Dist>    CircularRegion;
typedef std::vector<CircularRegion> CircularRegions;

int main(int argc, char** argv)
{
  RegisterThisFunction fnguard(__PRETTY_FUNCTION__);

  const char* progname = *argv;
  VSOptions options(argc, argv, true);

  bool print_usage = false;
  if(options.find("h","Print this message.")!=VSOptions::FS_NOT_FOUND)
    print_usage=true;
  if(options.find("help","Print this message.")!=VSOptions::FS_NOT_FOUND)
    print_usage=true;

  Config C;
  C.configure(options);

  unsigned                        nsample = 1;
  unsigned                        sleepsec = 600;
  std::string                     power_down_axes = "xz";

  options.findWithValue("nsample",nsample,
			"Set the number of measurement to average over at "
			"each grid point.");
  options.findWithValue("sleep",sleepsec,
			"Specify the number of seconds to sleep between "
			"measurement sets.");
  options.findWithValue("power_down_axes",power_down_axes,
			"Set the probe axes for which the stages should be "
			"powered down when the scan is finished. For example "
			"\"xz\" powers down the stages for the X and Z axes but"
			"leaves the Y-axis stage energized.");

  if(!options.assertNoOptions())
    {
      std::cerr << progname << ": unknown options: ";
      for(int i=1;i<argc;i++)
	if(*(argv[i])=='-') std::cerr << ' ' << argv[i];
      std::cerr << std::endl;
      usage(progname, options, std::cerr);
      exit(EXIT_FAILURE);
    }

  argv++,argc--;

  if(print_usage)
    {
      usage(progname, options, std::cout);
      exit(EXIT_SUCCESS);
    }

  if(argc==0)
    {
      usage(progname, options, std::cout);
      exit(EXIT_FAILURE);
    }

  // Parse the "power_down_axes" option now for safety
  for(std::string::iterator ichar = power_down_axes.begin();
      ichar != power_down_axes.end(); ichar++)
    {
      if(*ichar != 'x' && *ichar != 'X' && 
	 *ichar != 'y' && *ichar != 'Y' &&
	 *ichar != 'z' && *ichar != 'Z')
	{
	  std::cerr << progname
		    << ": unknown character in power_down_axes option: "
		    << *ichar << std::endl;
	  usage(progname, options, std::cerr);
	  exit(EXIT_FAILURE);
	}
    }

  std::vector<Coord> xy;
  xy.reserve(argc);
  while(argc)
    {
      Coord _xy;
      if(!VSDataConverter::fromString(_xy,*argv))
	{
	  std::cerr << "Could not convert \"" << *argv << "\" to x/y values\n";
	  std::cerr << std::endl;
	  usage(progname, options, std::cerr);
	  exit(EXIT_FAILURE);
	}
      xy.push_back(_xy);
      argv++,argc--;
    }

  DataStream* probe_ds = 0;
  ESPProtocol* probe_esp = 0;

  DataStream* mirror_ds = 0;
  ESPProtocol* mirror_esp = 0;

  try
    {
      probe_ds   = ESPProtocol::makeSerialDataStream(C.dev_probe);
      probe_esp  = new ESPProtocol(probe_ds);
      mirror_ds  = ESPProtocol::makeSerialDataStream(C.dev_mirror);
      mirror_esp = new ESPProtocol(mirror_ds);
    }
  catch(const CommunicationError& x)
    {
      x.print(std::cerr);
      std::cerr << strerror(x.errorNum()) << '\n';
      exit(EXIT_FAILURE);
    }

  std::cerr << std::fixed << std::setprecision(5);

  try
    {
      probe_esp->clearAllErrorCodes();
      mirror_esp->clearAllErrorCodes();

      if(isProbeTriggered(probe_esp,C))
	{
	  std::cerr << 
"Probe is triggered at start of data taking which indicates that it is\n"
"touching mirror or is perhaps disconnected from the ESP-300. Please move\n"
"probe off of surface and/or check connection to ESP-300. Aborting.\n";
	  throw Abort();
	}

      probe_esp->cmdMotorOn(C.ax_probe_x);
      usleep(100000);
      probe_esp->cmdMotorOn(C.ax_probe_y);
      usleep(100000);
      probe_esp->cmdMotorOn(C.ax_probe_z);
      usleep(100000);
      mirror_esp->cmdMotorOn(C.ax_mirror_x);
      usleep(100000);
      mirror_esp->cmdMotorOn(C.ax_mirror_z);
      usleep(100000);
      mirror_esp->cmdMotorOn(C.ax_mirror_t);
      usleep(100000);

      probe_esp->setEnableDIOToInhibitMotion(C.ax_probe_x, false);
      probe_esp->setEnableDIOToInhibitMotion(C.ax_probe_y, false);
      probe_esp->setEnableDIOToInhibitMotion(C.ax_probe_z, false);

      Dist x0; probe_esp->getActualPosition(C.ax_probe_x, x0);
      Dist y0; probe_esp->getActualPosition(C.ax_probe_y, y0);
      Dist z0; probe_esp->getActualPosition(C.ax_probe_z, z0);

      Dist x = x0;
      Dist y = y0;
      Dist z = z0;

      std::vector<std::pair<bool,Dist> > xy_z(xy.size());

      unsigned iloop = 0;
      while(1)
	{
	  unsigned ixy = iloop%xy.size();
	  Dist test_x = xy[ixy].first;
	  Dist test_y = xy[ixy].second;

	  Dist z_est  = C.limit_abs_z_bwd;
	  Dist z_bwd  = C.limit_abs_z_bwd;
	  Dist z_fwd  = C.limit_abs_z_bwd;
	  Dist z_slew = C.limit_abs_z_bwd;

	  if(xy_z[ixy].first)
	    {
	      z_est = xy_z[ixy].second;
	      z_bwd = 
		z_est - C.seek_step_big*(double(C.seek_nstep_big_back)+0.8);
	      z_fwd = z_est + C.limit_rel_z;
	    }
	  else
	    {
	      z_est = C.limit_abs_z_fwd;
	      z_bwd = C.limit_abs_z_bwd;
	      z_fwd = C.limit_abs_z_fwd;
	    }

	  assert(z_bwd >= z_slew);
	  assert(z_fwd >= z_bwd);

	  if(z_slew < C.limit_abs_z_bwd)
	    {
	      std::cerr
<< "Z-axis backward limit would be exceeded when retracting probe.. aborting"
<< std::endl;
	      throw Abort();
	    }

	  if(z_fwd > C.limit_abs_z_fwd)
	    {
	      std::cerr
<< "Z-axis forward limit would be exceeded when positioning probe.. aborting"
<< std::endl;
	      throw Abort();
	    }

#if 0
	  std::cerr << z_est << ' ' << z_bwd << ' ' << z_fwd << ' ' 
		    << test_y << ' ' << test_x 
		    << std::endl;
#endif

	  z = z_slew;
	  if(!probe_esp->cmdErrorFreeMoveToAbsolutePosition(C.ax_probe_z, z))
	    {
	      std::cerr
		<< "Error on Z-axis negative slew.. aborting"
		<< std::endl;
	      throw Abort();
	    }

	  if(ixy==0 && iloop>0)
	    sleep(sleepsec);

	  if(y != test_y)
	    {
	      y = test_y;
	      if(!probe_esp->cmdProtectedMoveToAbsolutePosition(C.ax_probe_y,
					  C.ibit_hit, y, C.bit_hit_value))
		{
		  std::cerr
		    << "Y-axis motion interrupted by interlock.. aborting"
		    << std::endl;
		  throw Abort();
		}
	    }

	  if(x != test_x)
	    {
	      x = test_x;
	      if(!probe_esp->cmdProtectedMoveToAbsolutePosition(C.ax_probe_x,
					  C.ibit_hit, x, C.bit_hit_value))
		{
		  std::cerr 
		    << "X-axis motion interrupted by interlock.. aborting"
		    << std::endl;
		  throw Abort();
		}
	    }

	  z = z_bwd;
	  if(!probe_esp->cmdProtectedMoveToAbsolutePosition(C.ax_probe_z,
					  C.ibit_hit, z, C.bit_hit_value))
	    {
	      std::cerr
		<< "Z-axis forward slew interrupted by interlock.. aborting"
		<< std::endl;
	      throw Abort();
	    }

	  while((z < z_fwd)&&(!isProbeTriggered(probe_esp,C)))
	    {
	      z += C.seek_step_big;
	      if(z > z_fwd)z = z_fwd;

	      if(!probe_esp->
		 cmdErrorFreeMoveToAbsolutePosition(C.ax_probe_z, z))
		{
		  std::cerr
		    << "Error on Z-axis large-step forward motion.. aborting"
		    << std::endl;
		  throw Abort();
		}
	    }

	  ESPProtocol::Dist z_hit = z;
	  if(z==z_fwd)
	    continue;

	  VSSimpleStat2<Dist> stat;
	  std::vector<Dist> z_sample; z_sample.reserve(nsample);
	  
	  bool retracted = false;
	  for(unsigned isample=0;isample<nsample;isample++)
	    {
	      while((!retracted)||(isProbeTriggered(probe_esp,C)))
		{
		  z -= C.seek_step_small*C.seek_nstep_small_back;
		  if(z < C.limit_abs_z_bwd)
		    {
		      std::cerr
<< "Z-axis backward limit would be exceeded when retracting probe.. aborting"
<< std::endl;
		      throw Abort();
		    }

		  if(!probe_esp->
		     cmdErrorFreeMoveToAbsolutePosition(C.ax_probe_z, z))
		    {
		      std::cerr
			<< "Error on Z-axis negtive motion.. aborting"
			<< std::endl;
		      throw Abort();
		    }

		  retracted = true;
		}

	      while(!isProbeTriggered(probe_esp,C))
		{
		  z += C.seek_step_small;
		  z = round(z/C.seek_step_small)*C.seek_step_small;
		  if(z > z_fwd)
		    {
		      std::cerr
<< "Z-axis forward limit would be exceeded when scanning probe.. aborting"
<< std::endl;
		      throw Abort();
		    }

		  if(!probe_esp->
		     cmdErrorFreeMoveToAbsolutePosition(C.ax_probe_z, z))
		    {
		      std::cerr
			<< "Error on Z-axis small-step forward motion.. aborting"
			<< std::endl;
		      throw Abort();
		    }
		}

	      retracted = false;

	      stat.accumulate(z);
	      z_sample.push_back(z);
	    }

	  time_t tnow = time(0);
	  std::cerr << tnow << ' ' << ixy << ' '
		    << x << ' ' << y << ' '
		    << stat.mean() << ' ' << stat.dev() << ' '
		    << median(z_sample) << ' ' << z_est << ' ' << z_hit;
	  
	  for(unsigned isample=0;isample<nsample;isample++)
	    std::cerr << ' ' << z_sample[isample];
	  std::cerr << '\n';

	  xy_z[ixy].first = true;
	  xy_z[ixy].second = stat.mean();

	  iloop++;
	}
    }
  catch(const CommunicationError& x)
    {
      x.print(std::cerr);
      std::cerr << strerror(x.errorNum()) << '\n';
    }
  catch(const Abort& x)
    {

    }

  // retract probe back to z=0 when finished (added 10/01/2014)
  probe_esp->cmdMoveToAbsolutePosition(C.ax_probe_z,0);
  probe_esp->pollForMotionDone(C.ax_probe_z);

  // Allow user to choose which axes to switch off to protect in case
  // of high gravity load (added 10/01/2014 and modified 10/11/2014)
  for(std::string::iterator ichar = power_down_axes.begin();
      ichar != power_down_axes.end(); ichar++)
    {
      if(*ichar == 'x' || *ichar == 'X')
	probe_esp->cmdMotorOff(C.ax_probe_x);
      else if(*ichar == 'y' || *ichar == 'Y')
	probe_esp->cmdMotorOff(C.ax_probe_y);
      else if(*ichar == 'z' || *ichar == 'Z')
	probe_esp->cmdMotorOff(C.ax_probe_z);
      else 
	std::cerr 
	  << progname << ": program logic error!!!" << std::endl
	  << progname
	  << ": unknown character in power_down_axes option: "
	  << *ichar << std::endl
	  << progname << ": should have been caught before scanning started." 
	  << std::endl;
    }
  mirror_esp->cmdMotorOff(C.ax_mirror_x);
  mirror_esp->cmdMotorOff(C.ax_mirror_z);
  mirror_esp->cmdMotorOff(C.ax_mirror_t);

  delete probe_esp;
  delete probe_ds;
  delete mirror_esp;
  delete mirror_ds;
}
