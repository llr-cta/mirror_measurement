//-*-mode:c++; mode:font-lock;-*-

#include<cmath>
#include<map>

#include<time.h>
#include<math.h>
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
    dev_probe("/dev/ttyUSB0"), ax_probe_x(0), ax_probe_y(1), ax_probe_z(2),
    ibit_hit(0), ibit_alive(1), bit_hit_value(true), bit_alive_value(true),
    dev_mirror("/dev/ttyUSB1"), ax_mirror_x(0), ax_mirror_z(1), ax_mirror_t(2),
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

class ScanSurfaceEstimator
{
public:
  typedef ESPProtocol::Dist Dist;
  virtual ~ScanSurfaceEstimator();
  virtual int guessZ(int i, int j, Dist& z_est) = 0;
  virtual void addMeasuredZ(int i, int j, Dist z_meas) = 0;
};

ScanSurfaceEstimator::~ScanSurfaceEstimator()
{
  // nothing to see here
}

class SimpleSurfaceEstimator: public ScanSurfaceEstimator
{
public:
  SimpleSurfaceEstimator(Dist initial_z):
    ScanSurfaceEstimator(), m_last_z(initial_z) { }
  virtual ~SimpleSurfaceEstimator();
  virtual int guessZ(int i, int j, Dist& z_est);
  virtual void addMeasuredZ(int i, int j, Dist z_meas);
private:
  typedef std::pair<int,int> IJ;

  bool extrapolate1d2(int i, int j, int di, int dj, Dist& z) const
  {
    Map::const_iterator iz1 = m_map.find(IJ(i+1*di,j+1*dj));
    Map::const_iterator iz2 = m_map.find(IJ(i+2*di,j+2*dj));
    Map::const_iterator iz3 = m_map.find(IJ(i+3*di,j+3*dj));
    if(iz1==m_map.end() || iz2==m_map.end() || iz3==m_map.end())return false;

    const Dist z1 = iz1->second;
    const Dist z2 = iz2->second;
    const Dist z3 = iz3->second;

    z = 3.0*z1 - 3.0*z2 + z3;
    return true;
  }

  bool extrapolate1d1(int i, int j, int di, int dj, Dist& z) const
  {
    Map::const_iterator iz1 = m_map.find(IJ(i+1*di,j+1*dj));
    Map::const_iterator iz2 = m_map.find(IJ(i+2*di,j+2*dj));
    if(iz1==m_map.end() || iz2==m_map.end())return false;

    const Dist z1 = iz1->second;
    const Dist z2 = iz2->second;

    z = 2.0*z1 - z2;
    return true;
  }

  bool extrapolate2d1(int i, int j, int di, int dj, Dist& z) const
  {
    Map::const_iterator iz11 = m_map.find(IJ(i+1*di,j+1*dj));
    Map::const_iterator iz01 = m_map.find(IJ(i+0*di,j+1*dj));
    Map::const_iterator iz10 = m_map.find(IJ(i+1*di,j+0*dj));
    if(iz11==m_map.end() || iz10==m_map.end() || iz01==m_map.end())
      return false;

    const Dist z11 = iz11->second;
    const Dist z01 = iz01->second;
    const Dist z10 = iz10->second;

    z = z10 + z01 - z11;
    return true;
  }

  struct IJCmp
  {
    bool operator()(const IJ& a, const IJ& b) const
    {
      return (a.first<b.first)||((a.first==b.first)&(a.second<b.second));
    }
  };

  typedef std::map<IJ,Dist,IJCmp> Map;

  Dist m_last_z;
  Map m_map;
};

SimpleSurfaceEstimator::~SimpleSurfaceEstimator()
{
  // nothing to see here
}

int SimpleSurfaceEstimator::guessZ(int i, int j, Dist& z_est)
{
  unsigned n_sum = 0;
  Dist z_one = 0;
  Dist z_sum = 0;

  if(extrapolate1d2(i, j,  1,  0, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d2(i, j,  1,  1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d2(i, j,  0,  1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d2(i, j, -1,  1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d2(i, j, -1,  0, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d2(i, j, -1, -1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d2(i, j,  0, -1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d2(i, j,  1, -1, z_one))z_sum += z_one, n_sum++;
  if(n_sum)
    {
      z_est = z_sum/Dist(n_sum);
      return n_sum;
    }

  if(extrapolate1d1(i, j,  1,  0, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d1(i, j,  1,  1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d1(i, j,  0,  1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d1(i, j, -1,  1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d1(i, j, -1,  0, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d1(i, j, -1, -1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d1(i, j,  0, -1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate1d1(i, j,  1, -1, z_one))z_sum += z_one, n_sum++;
  if(n_sum)
    {
      z_est = z_sum/Dist(n_sum);
      return n_sum;
    }

  if(extrapolate2d1(i, j,  1,  1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate2d1(i, j, -1,  1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate2d1(i, j, -1, -1, z_one))z_sum += z_one, n_sum++;
  if(extrapolate2d1(i, j,  1, -1, z_one))z_sum += z_one, n_sum++;
  if(n_sum)
    {
      z_est = z_sum/Dist(n_sum);
      return n_sum;
    }

  if(!m_map.empty())
    {
      Map::const_iterator imeas = m_map.begin();
      const IJ* ij = &imeas->first;
      Dist di = Dist(ij->first-i);
      Dist dj = Dist(ij->second-j);
      Dist d2_min = SQR(di)+SQR(dj);
      Dist z_min = imeas->second;
      imeas++;
      while(imeas != m_map.end())
  {
    ij = &imeas->first;
    di = Dist(ij->first-i);
    dj = Dist(ij->second-j);
    Dist d2 = SQR(di)+SQR(dj);
    if(d2 < d2_min)d2_min=d2, z_min=imeas->second;
    imeas++;
  }
      z_est = z_min;
      return 1;
    }

  z_est = m_last_z;
  return 0;
}

void SimpleSurfaceEstimator::addMeasuredZ(int i, int j, Dist z_meas)
{
  m_map[IJ(i,j)] = z_meas;
  m_last_z = z_meas;
}

void usage(const std::string& progname, const VSOptions& options,
           std::ostream& stream)
{
  stream << "Usage: " << progname  << " [options] [--] x_range y_range\n\n"
   << "Where x_range and y_range specify the size of the scanned region in mm\n\nNOTE: To specify a negative value of x_range or y_range you MUST terminate the\noptions processing using a double hyphen (--) before x_range, otherwise the\nthe negative value will be treated as an option.\n\n";
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

bool isInExclusionRegion(Dist x, Dist y, const CircularRegions& cregions)
{
  for(CircularRegions::const_iterator iregion=cregions.begin();
      iregion!=cregions.end(); iregion++)
    {
      Dist xc = x-iregion->first;
      Dist yc = y-iregion->second;
      Dist rc2 = SQR(xc)+SQR(yc);
      Dist rr2 = SQR(iregion->third);
      if(rc2 < rr2)return true;
    }
  return false;
}

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

  Dist                            dxdy = 10.0;
  Dist                            dx   = 0.0;
  Dist                            dy   = 0.0;
  unsigned                        nsample = 1;
  unsigned                        calib_interval = 0;
  std::vector<Coord>              calib_xy;
  Dist                            zzero = 0;
  bool                            zzero_set = 0;
  CircularRegions                 exclusions;
  std::string                     power_down_axes = "uvwxz";

  options.findWithValue("res",dxdy,"Set the scan grid in mm.");
  options.findWithValue("res_x",dx,"Set the x-axis scan grid in mm. If "
  "non-zero, this option overrides the value set by "
  "\"res\".");
  options.findWithValue("res_y",dy,"Set the y-axis scan grid in mm. If "
  "non-zero, this option overrides the value set by "
  "\"res\".");
  options.findWithValue("nsample",nsample,
  "Set the number of measurement to average over at "
  "each grid point.");
  options.findWithValue("calib_interval",calib_interval,
  "Specify the number of scan points to take between "
  "each calibration measurement. A value of zero "
  "disables calibration");
  options.findWithValue("calib_xy",calib_xy,
  "Set the coordinates (X,Y in mm) of the calibration "
  "points. Enter as a comma separeted list of \"x/y\" "
  "points, for example \"0/0,580/0,300/380,300/120\".");
  if(options.findWithValue("zzero",zzero,
     "Set the initial guess at the surface position. If "
     "not set, then the current z coordinate is used.")
     == VSOptions::FS_FOUND)zzero_set = true;
  options.findWithValue("exclusions",exclusions,
  "Define a list of circular exclusion regions. Should "
  "be provided as a comma separated list of triple "
  "distance values (in mm): x/y/r. For example "
  "\"10/20/5,40/50/10\".");
  options.findWithValue("power_down_axes",power_down_axes,
  "Set the probe axes for which the stages should be "
  "powered down when the scan is finished. For example "
  "\"xz\" powers down the stages for the X and Z axes but"
  "leaves the Y-axis stage energized. Additionally, axes on "
      "the secondary controller can be given as \"uvw\".");

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

  if(argc!=2)
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
        *ichar != 'z' && *ichar != 'Z' &&
        // 2018-06-21 Borreze France : Add UVW to control power on axes of
        // secondary controller
        *ichar != 'u' && *ichar != 'U' &&
        *ichar != 'v' && *ichar != 'V' &&
        *ichar != 'w' && *ichar != 'W')
      {
        std::cerr << progname
          << ": unknown character in power_down_axes option: "
          << *ichar << std::endl;
        usage(progname, options, std::cerr);
        exit(EXIT_FAILURE);
      }
    }

  Dist x_range = 1.0;
  VSDataConverter::fromString(x_range,*argv);
  argv++,argc--;

  Dist y_range = 1.0;
  VSDataConverter::fromString(y_range,*argv);
  argv++,argc--;

  if(dx == 0)dx=dxdy;
  if(dy == 0)dy=dxdy;
  if(x_range < 0)dx = -std::abs(dx);
  if(y_range < 0)dy = -std::abs(dy);

  unsigned nx = lround(std::abs(x_range)/std::abs(dx))+1;
  unsigned ny = lround(std::abs(y_range)/std::abs(dy))+1;

  std::cout << x_range << ' ' << dx << ' ' << nx << ' '
      << y_range << ' ' << dy << ' ' << ny << '\n';

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

#if 0
      for(std::string::iterator ichar = power_down_axes.begin();
          ichar != power_down_axes.end(); ichar++)
        {
          if(*ichar == 'x' || *ichar == 'X')
        	  continue;
          else if(*ichar == 'y' || *ichar == 'Y')
            continue;
          else if(*ichar == 'z' || *ichar == 'Z')
            continue;
          // 2018-06-21 Borreze France : Add UVW to control power on axes of
          // secondary controller - only energise them if they will be powered
          // off
          else if(*ichar == 'u' || *ichar == 'U') {
            mirror_esp->cmdMotorOn(C.ax_mirror_x);
            usleep(100000);
          }
          else if(*ichar == 'v' || *ichar == 'V') {
            mirror_esp->cmdMotorOn(C.ax_mirror_z);
            usleep(100000);
          }
          else if(*ichar == 'w' || *ichar == 'W') {
            mirror_esp->cmdMotorOn(C.ax_mirror_t);
            usleep(100000);
          }
          else
          	std::cerr
          	  << progname << ": program logic error!!!" << std::endl
          	  << progname
          	  << ": unknown character in power_down_axes option: "
          	  << *ichar << std::endl
          	  << progname << ": should have been caught before scanning started."
          	  << std::endl;
        }
#else
      mirror_esp->cmdMotorOn(C.ax_mirror_x);
      usleep(100000);
      mirror_esp->cmdMotorOn(C.ax_mirror_z);
      usleep(100000);
      mirror_esp->cmdMotorOn(C.ax_mirror_t);
      usleep(100000);
#endif

      probe_esp->setEnableDIOToInhibitMotion(C.ax_probe_x, false);
      probe_esp->setEnableDIOToInhibitMotion(C.ax_probe_y, false);
      probe_esp->setEnableDIOToInhibitMotion(C.ax_probe_z, false);

      Dist x0; probe_esp->getActualPosition(C.ax_probe_x, x0);
      Dist y0; probe_esp->getActualPosition(C.ax_probe_y, y0);
      Dist z0; probe_esp->getActualPosition(C.ax_probe_z, z0);

      if(!zzero_set)zzero=z0;
      ScanSurfaceEstimator* estimator = new SimpleSurfaceEstimator(zzero);

      if(calib_interval>0 && calib_xy.empty())calib_xy.push_back(Coord(x0,y0));

      unsigned loop_iy = 0;
      unsigned loop_ix = 0;

      Dist x = x0;
      Dist y = y0;
      Dist z = z0;

      std::vector<std::vector<Dist> > calib_z(calib_xy.size());

      unsigned iscan = 0;
      unsigned ncal = 0;
      bool calibration_last = false;
      while(loop_iy<ny
      || ((calib_interval>0)&&((calibration_last==false)||(ncal>0))))
  {
    unsigned iix = loop_ix;
    if(loop_iy%2==1)iix = nx-loop_ix-1;

    Dist test_y = y0+loop_iy*dy;
    Dist test_x = x0+iix*dx;

    if((calib_interval>0)&&((loop_iy>=ny)||(iscan%calib_interval==0))
       &&(calibration_last==false)&&(ncal==0))
      ncal = calib_xy.size();

    bool calibration = (calib_interval>0)&&(ncal>0);

    unsigned ical = calib_xy.size();
    if(calibration)
      {
        ical = calib_xy.size() - ncal;
        ncal--;
        test_x = calib_xy[ical].first;
        test_y = calib_xy[ical].second;
      }
    else
      {
        iscan++;
      }

    bool excluded = isInExclusionRegion(test_x,test_y,exclusions);

    Dist z_est  = C.limit_abs_z_bwd;
    Dist z_bwd  = C.limit_abs_z_bwd;
    Dist z_fwd  = C.limit_abs_z_bwd;
    Dist z_slew = C.limit_abs_z_bwd;

    if(excluded)
      {
        z_est  = C.limit_abs_z_bwd;
        z_bwd  = C.limit_abs_z_bwd;
        z_fwd  = C.limit_abs_z_bwd;
        z_slew = C.limit_abs_z_bwd;
      }
    else
      {
        int n_est = 0;
        if(calibration)
  {
    if(!calib_z[ical].empty())
      {
        z_est = median(calib_z[ical]);
        n_est = calib_z[ical].size();
      }
    else
      {
        z_est = C.limit_abs_z_fwd;
        n_est = -1;
      }
  }
        else
  {
    n_est = estimator->guessZ(iix,loop_iy, z_est);
    if(n_est<0)assert(0);
  }

        if((calibration)&&(calib_z[ical].empty()))
  z_bwd = C.limit_abs_z_bwd;
        else
  z_bwd =
    z_est - C.seek_step_big*(double(C.seek_nstep_big_back)+0.8);

        Dist z_safe = z - C.seek_step_big*double(C.seek_nstep_big_back);
        if(z_safe < C.limit_abs_z_bwd)z_safe = C.limit_abs_z_bwd;

        if((calibration)||(calibration_last))z_slew = C.limit_abs_z_bwd;
        else if(z_bwd > z_safe)z_slew = z_safe;
        else z_slew = z_bwd;

        if(n_est > 0)
  z_fwd = z_est + C.limit_rel_z;
        else if(n_est == 0)
  z_fwd = z_est + C.limit_rel_z*C.limit_rel_z_zero_est_mult;
        else
  z_fwd = C.limit_abs_z_fwd;
      }

    calibration_last = calibration;

    if((y == test_y)&&(x == test_x))goto skip_motion;

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
        if(C.limit_abs_z_fwd > 9998.5)
  {
    // User did not set fwd limit so die here to avoid smashing
    // probe
    std::cerr
      << "Z-axis forward limit would be exceeded when positioning probe.. aborting"
<< std::endl;
    throw Abort();
  }
        else
  {
    // Otherwise trust that they know what they are doing!
    z_fwd = C.limit_abs_z_fwd;
  }
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

    if(excluded)
      {
        if(!calibration)
  {
    loop_ix++;
    if(loop_ix == nx)loop_ix=0,loop_iy++;
  }
        continue;
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

  skip_motion:

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
      {
        if(!calibration)
  {
    loop_ix++;
    if(loop_ix == nx)loop_ix=0,loop_iy++;
  }
        continue;
      }

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
        if(calibration)calib_z[ical].push_back(z);
      }

    time_t tnow = time(0);
    std::cerr << tnow << ' ';

    if(calibration)
      std::cerr << 1 << ' ' << -1 << ' ' << ical << ' ';
    else
      std::cerr << 0 << ' ' << iix << ' ' << loop_iy << ' ';

    std::cerr << x << ' ' << y << ' '
      << stat.mean() << ' ' << stat.dev() << ' '
      << median(z_sample) << ' ' << z_est << ' ' << z_hit;

    for(unsigned isample=0;isample<nsample;isample++)
      std::cerr << ' ' << z_sample[isample];
    std::cerr << '\n';

    if(!calibration)
      {
        estimator->addMeasuredZ(iix,loop_iy,stat.mean());
        loop_ix++;
        if(loop_ix == nx)loop_ix=0,loop_iy++;
      }
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
      // 2018-06-21 Borreze France : Add UVW to control power on axes of
      // secondary controller
      else if(*ichar == 'u' || *ichar == 'U')
        mirror_esp->cmdMotorOff(C.ax_mirror_x);
      else if(*ichar == 'v' || *ichar == 'V')
        mirror_esp->cmdMotorOff(C.ax_mirror_z);
      else if(*ichar == 'w' || *ichar == 'W')
        mirror_esp->cmdMotorOff(C.ax_mirror_t);
      else
      	std::cerr
      	  << progname << ": program logic error!!!" << std::endl
      	  << progname
      	  << ": unknown character in power_down_axes option: "
      	  << *ichar << std::endl
      	  << progname << ": should have been caught before scanning started."
      	  << std::endl;
    }

  delete probe_esp;
  delete probe_ds;
  delete mirror_esp;
  delete mirror_ds;
}
