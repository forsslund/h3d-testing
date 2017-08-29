//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
//    Copyright 2008-2009, Kyle Machulis
//    Copyright 2009, Karljohan Lundin Palmerius
//
//    This file is part of HAPI.
//
//    HAPI is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    HAPI is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with HAPI; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file NiFalconHapticsDevice.cpp
/// \brief Cpp file for NiFalconHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#define NIFALCONAPI_LIBUSB
#include <HAPI/NiFalconHapticsDevice.h>

#ifdef HAVE_NIFALCONAPI

#include <falcon/core/FalconDevice.h>

#ifdef NIFALCONAPI_LIBFTD2XX
# include <falcon/comm/FalconCommFTD2XX.h>
#endif

#ifdef NIFALCONAPI_LIBFTDI
# include <falcon/comm/FalconCommLibFTDI.h>
#endif

#ifdef NIFALCONAPI_LIBUSB
# include <falcon/comm/FalconCommLibUSB.h>
#endif

#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>

#include <falcon/grip/FalconGripFourButton.h>

#include <H3DUtil/DynamicLibrary.h>

//------------------------------------------------------------------------------
// From CWoodenDevice.h
//------------------------------------------------------------------------------
// Following includes are only used for reading/writing config file and to find
// the user's home directory (where the config file will be stored)
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#define LAN
//------------------------------------------------------------------------------

//#include <boost/array.hpp>
//#include <boost/asio.hpp>

//using boost::asio::ip::udp;



using namespace HAPI;
using namespace libnifalcon;
using namespace std;










//------------------------------------------------------------------------------
// From CWoodenDevice.h
//------------------------------------------------------------------------------
std::string toJSON(const woodenhaptics_message& m) {
    std::stringstream ss;
    ss << "{" << std::endl <<
          "  'position_x':       " << m.position_x << "," << std::endl <<
          "  'position_y':       " << m.position_y << "," << std::endl <<
          "  'position_z':       " << m.position_z << "," << std::endl <<
          "  'command_force_x':  " << m.command_force_x << "," << std::endl <<
          "  'command_force_y':  " << m.command_force_y << "," << std::endl <<
          "  'command_force_z':  " << m.command_force_z << "," << std::endl <<
          "  'actual_current_0': " << m.actual_current_0 << "," << std::endl <<
          "  'actual_current_1': " << m.actual_current_1 << "," << std::endl <<
          "  'actual_current_2': " << m.actual_current_2 << "," << std::endl <<
          "  'temperture_0':     " << m.temperature_0 << "," << std::endl <<
          "  'temperture_1':     " << m.temperature_1 << "," << std::endl <<
          "  'temperture_2':     " << m.temperature_2 << "," << std::endl <<
          "}" << std::endl;

    return ss.str();
}
NiFalconHapticsDevice::configuration default_woody(){
    double data[] = { 0, 0.010, 0.010, 0.010,
                      0.080, 0.205, 0.245,
                      0.160, 0.120, 0.120,
                      0.220, 0.000, 0.080, 0.100,
                      0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,
                      5.0, 1000.0, 8.0,
                      0.170, 0.110, 0.051, 0.091, 0};
    return NiFalconHapticsDevice::configuration(data);
}

double v(const std::string& json, const std::string& key){
    int p = json.find(":", json.find(key));
    return atof(json.substr(p+1).c_str());
}

NiFalconHapticsDevice::configuration fromJSON(std::string json){
    double d[]= {
        v(json,"variant"),
        v(json,"diameter_capstan_a"),
        v(json,"diameter_capstan_b"),
        v(json,"diameter_capstan_c"),
        v(json,"length_body_a"),
        v(json,"length_body_b"),
        v(json,"length_body_c"),
        v(json,"diameter_body_a"),
        v(json,"diameter_body_b"),
        v(json,"diameter_body_c"),
        v(json,"workspace_origin_x"),
        v(json,"workspace_origin_y"),
        v(json,"workspace_origin_z"),
        v(json,"workspace_radius"),
        v(json,"torque_constant_motor_a"),
        v(json,"torque_constant_motor_b"),
        v(json,"torque_constant_motor_c"),
        v(json,"current_for_10_v_signal"),
        v(json,"cpr_encoder_a"),
        v(json,"cpr_encoder_b"),
        v(json,"cpr_encoder_c"),
        v(json,"max_linear_force"),
        v(json,"max_linear_stiffness"),
        v(json,"max_linear_damping"),
        v(json,"mass_body_b"),
        v(json,"mass_body_c"),
        v(json,"length_cm_body_b"),
        v(json,"length_cm_body_c"),
        v(json,"g_constant")
    };
    return NiFalconHapticsDevice::configuration(d);
}

std::string j(const std::string& key, const double& value){
    std::stringstream s;
    s << "    \"" << key << "\":";
    while(s.str().length()<32) s<< " ";
    s << value << "," << std::endl;
    return s.str();
}
std::string toJSON(const NiFalconHapticsDevice::configuration& c){
    using namespace std;
    stringstream json;
    json << "{" << endl
         << j("variant",c.variant)
         << j("diameter_capstan_a",c.diameter_capstan_a)
         << j("diameter_capstan_b",c.diameter_capstan_b)
         << j("diameter_capstan_c",c.diameter_capstan_c)
         << j("length_body_a",c.length_body_a)
         << j("length_body_b",c.length_body_b)
         << j("length_body_c",c.length_body_c)
         << j("diameter_body_a",c.diameter_body_a)
         << j("diameter_body_b",c.diameter_body_b)
         << j("diameter_body_c",c.diameter_body_c)
         << j("workspace_origin_x",c.workspace_origin_x)
         << j("workspace_origin_y",c.workspace_origin_y)
         << j("workspace_origin_z",c.workspace_origin_z)
         << j("workspace_radius",c.workspace_radius)
         << j("torque_constant_motor_a",c.torque_constant_motor_a)
         << j("torque_constant_motor_b",c.torque_constant_motor_b)
         << j("torque_constant_motor_c",c.torque_constant_motor_c)
         << j("current_for_10_v_signal",c.current_for_10_v_signal)
         << j("cpr_encoder_a",c.cpr_encoder_a)
         << j("cpr_encoder_b",c.cpr_encoder_b)
         << j("cpr_encoder_c",c.cpr_encoder_c)
         << j("max_linear_force",c.max_linear_force)
         << j("max_linear_stiffness",c.max_linear_stiffness)
         << j("max_linear_damping",c.max_linear_damping)
         << j("mass_body_b",c.mass_body_b)
         << j("mass_body_c",c.mass_body_c)
         << j("length_cm_body_b",c.length_cm_body_b)
         << j("length_cm_body_c",c.length_cm_body_c)
         << j("g_constant",c.g_constant)
         << "}" << endl;
    return json.str();
}

void write_config_file(const NiFalconHapticsDevice::configuration& config){
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    std::cout << "Writing configuration to: "<< homedir
              << "/woodenhaptics.json" << std::endl;
    std::ofstream ofile;
    ofile.open(std::string(homedir) + "/woodenhaptics.json");
    ofile << toJSON(config);
    ofile.close();
}

NiFalconHapticsDevice::configuration read_config_file(){
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    std::cout << "Trying loading configuration from: "<< homedir
              << "/woodenhaptics.json" << std::endl;

    std::ifstream ifile;
    ifile.open(std::string(homedir) + "/woodenhaptics.json");
    if(ifile.is_open()){
        std::stringstream buffer;
        buffer << ifile.rdbuf();
        ifile.close();
        std::cout << "Success. " << std::endl;
        return fromJSON(buffer.str());
    } else {
        std::cout << "File not found. We will write one "
                  << "based on default configuration values." << std::endl;

        write_config_file(default_woody());
        return default_woody();
    }
}




const double pi = 3.14159265359;

//==============================================================================
// Helper functions for getPosition & setForce
//==============================================================================

struct pose {
    double Ln;
    double Lb;
    double Lc;
    double tA;  // angle of body A (theta_A)
    double tB;  // angle of body B (theta_B)
    double tC;  // angle of body C (theta_C)
};

pose calculate_pose(const NiFalconHapticsDevice::configuration& c, double* encoder_values) {
    pose p;

    double cpr[] = { c.cpr_encoder_a, c.cpr_encoder_b, c.cpr_encoder_c };
    double gearRatio[] = { c.diameter_body_a / c.diameter_capstan_a,
                           -c.diameter_body_b / c.diameter_capstan_b,
                           c.diameter_body_c / c.diameter_capstan_c };

    double dofAngle[3];
#if defined(USB) || defined(LAN)
    for(int i=0;i<3;i++)
        dofAngle[i] = (2.0*pi*encoder_values[i]/cpr[i]) / gearRatio[i];
#else
    for(int i=0;i<3;i++)
        dofAngle[i] = getMotorAngle(i,cpr[i]) / gearRatio[i];
    dofAngle[0] = -dofAngle[0]; // 2016-04-25 sign switch
#endif

    if(int(c.variant) == 1){ // ALUHAPTICS
        dofAngle[0] = -dofAngle[0];
        dofAngle[1] = dofAngle[1];
        dofAngle[2] = dofAngle[2];
    }

    // Calculate dof angles (theta) for each body
    p.Ln = c.length_body_a;
    p.Lb = c.length_body_b;
    p.Lc = c.length_body_c;
    p.tA = dofAngle[0];
    p.tB = dofAngle[1];
    p.tC = dofAngle[2]; // 2016-05-30

    return p;
}

double deg(double rad){
    return 360*rad/(2*3.141592);
}

//------------------------------------------------------------------------------
























//#define DEBUG_PRINTOUTS

namespace NiFalconHapticsDeviceInternal {
  list< string > nifalcon_device_libs;
}

HAPIHapticsDevice::HapticsDeviceRegistration 
NiFalconHapticsDevice::device_registration
( "NiFalconHapticsDevice",
  &(newInstance< NiFalconHapticsDevice >),
  NiFalconHapticsDeviceInternal::nifalcon_device_libs );

/// Constructor. device_index is the index of falcon device
/// connected. Should not be larger than getNrConnectedFalconDevices() - 1.
NiFalconHapticsDevice::NiFalconHapticsDevice( unsigned int device_index ):
  com_thread( NULL ),
  com_func_cb_handle( -1 ),
  index( device_index ),
  device( new libnifalcon::FalconDevice ), totalTime(0) {
  
  max_stiffness = 800;
}

NiFalconHapticsDevice::~NiFalconHapticsDevice() {
  delete device;
}

bool NiFalconHapticsDevice::initHapticsDevice( int _thread_frequency ) {

    m_config = read_config_file();

    int port = 47111;
    std::cout << "Open port " << port << " for Remote Haptics" << std::endl;
    rh.init(port);













  
#ifdef NIFALCONAPI_LIBUSB
  //device->setFalconComm<libnifalcon::FalconCommLibUSB>();
#else
#ifdef NIFALCONAPI_LIBFTD2XX
  device->setFalconComm<libnifalcon::FalconCommFTD2XX>();
#else
#ifdef NIFALCONAPI_LIBFTDI
  device->setFalconComm<libnifalcon::FalconCommLibFTDI>();
#endif
#endif
#endif
  
  //device->setFalconFirmware<libnifalcon::FalconFirmwareNovintSDK>();
  //device->setFalconKinematic<libnifalcon::FalconKinematicStamper>();
  //device->setFalconGrip<libnifalcon::FalconGripFourButton>();
 
#ifdef DEBUG_PRINTOUTS
  // get number of connected falcons.
  unsigned int num_falcons;
#endif

  
#ifdef DEBUG_PRINTOUTS
  if(!device->getDeviceCount(num_falcons)) {
    H3DUtil::Console(LogLevel::Error) << "Cannot get device count" << std::endl;
  }

  H3DUtil::Console(LogLevel::Error) << "Falcons found: " << num_falcons << std::endl;

  std::cout << "Opening falcon " << index << std::endl;

#endif

  /*

  if(!device->open( index )) {
    stringstream s;
    s << "Cannot open falcon(index " << index << ") - Error: " 
      << device->getErrorCode() 
      << ". Make sure you have the device connected properly "
      << "and have the permissions to communicate over the USB "
      << "port. " << std::endl;
    setErrorMsg( s.str() );
    return false;
  }

  if(!device->isFirmwareLoaded()) {
    bool has_firmware = false;
    for(int i = 0; i < 10; ++i) {
      if(!device->getFalconFirmware()->
        loadFirmware(true, 
                     NOVINT_FALCON_NVENT_FIRMWARE_SIZE,
                     const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
        {
#ifdef DEBUG_PRINTOUTS
    std::cout << "Cannot load firmware" << std::endl;
#endif
        }
      else
        {
    has_firmware = true;
#ifdef DEBUG_PRINTOUTS
    std::cout <<"firmware loaded" << std::endl;
#endif
    break;
        }
    }

    if(!has_firmware) {
      setErrorMsg( "Could not load Falcon firmware(libnifalcon)." );
      return false;
    }
  }
  */

  // set up a communication thread for the device.
  com_thread = 
    new H3DUtil::PeriodicThread( H3DUtil::ThreadBase::HIGH_PRIORITY, 1000 );
  com_thread->setThreadName( "NiFalconHapticsDevice com thread" );
  
  com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );

  return true;
}

bool NiFalconHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  
  if( com_thread ) {
    if( com_func_cb_handle != -1 ) {
      com_thread->removeAsynchronousCallback( com_func_cb_handle );
      com_func_cb_handle = -1;
    }
    delete com_thread;
    com_thread = NULL;
  }
  
  return true;
}

void NiFalconHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                                HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  com_lock.lock();
  totalTime+=dt;


  Libremotehaptics::Vector3d lp;
  rh.getPosition(lp);
  double encoder_values[] = { -lp.x,
                               lp.y,
                               lp.z };
  pose p  = calculate_pose(m_config, encoder_values);



  const double& Ln = p.Ln;
  const double& Lb = p.Lb;
  const double& Lc = p.Lc;
  const double& tA = p.tA;
  double tB = p.tB;
  double tC = p.tC;

  // Mike edition
  if(int(m_config.variant) == 1) // ALUHAPTICS
      tB = tB + 3.141592/2;
  else
      tC = -tC + 3.141592/2;

  double x,y,z;
  x = cos(tA)*(Lb*sin(tB)+Lc*sin(tC))    - m_config.workspace_origin_x;
  y = sin(tA)*(Lb*sin(tB)+Lc*sin(tC)) - m_config.workspace_origin_y;
  z = Ln+Lb*cos(tB)-Lc*cos(tC) - m_config.workspace_origin_z;


  dv.position = Vec3(y,z,x); // Chai->H3D transform
  //dv.position = Vec3(0.1*sin(totalTime),p.y,0.05+p.z);//current_values.position;

  std::cout << "Got from remotehaptics: " << lp.x << "," << lp.y << "," << lp.z << "\n";
  std::cout << "Calculated position (chai): " << x << "," << y << "," << z << "\n";








  calculateVelocity(dv, dt);
  dv.orientation = current_values.orientation;
  dv.button_status = current_values.button_status;
  com_lock.unlock();
}

void NiFalconHapticsDevice::sendOutput( DeviceOutput &dv,
                                        HAPITime dt ) {
  com_lock.lock();
  current_values.force = dv.force;
  //std::cout << "Force " << dv.force.x << " "<< dv.force.y << " "<< dv.force.z << " " << "\n";

  Libremotehaptics::Vector3d lp;
  rh.getPosition(lp);
  double encoder_values[] = { -lp.x,
                               lp.y,
                               lp.z };
  pose p  = calculate_pose(m_config, encoder_values);
  const double& Ln = p.Ln;
  const double& Lb = p.Lb;
  const double& Lc = p.Lc;
  const double& tA = p.tA;
  double tB = p.tB;
  double tC = p.tC;

  // Mike edition
  if(int(m_config.variant) == 1) // ALUHAPTICS
      tB = tB + 3.141592/2;
  else
      tC = -tC + 3.141592/2;
  Matrix3 J(  -sin(tA)*(Lb*sin(tB)+Lc*sin(tC)),    Lb*cos(tA)*cos(tB),   Lc*cos(tA)*cos(tC),
          cos(tA)*(Lb*sin(tB)+Lc*sin(tC)),    Lb*sin(tA)*cos(tB),   Lc*sin(tA)*cos(tC),
          0,                           -Lb*sin(tB),           Lc*sin(tC)     );
  Matrix3 H3DtoChai(0,0,1,
                    1,0,0,
                    0,1,0);
  Vec3 f = H3DtoChai * dv.force;
  Vec3 t = J.transpose() * f;

  // Gravity compensation
  const double& g=m_config.g_constant;
  const double& Lb_cm = m_config.length_cm_body_b;
  const double& Lc_cm = m_config.length_cm_body_c;
  const double& mB = m_config.mass_body_b;
  const double& mC = m_config.mass_body_c;

  t = t + -g*Vec3( 0,
                   mB*Lb_cm*sin(tB) + mC*(Lb_cm + Lc_cm)*sin(tC),
                   mC*Lc_cm*sin(tC) );
  // Gear down
  double motorTorque[] = {
      t.x * m_config.diameter_capstan_a / m_config.diameter_body_a,
      -t.y * m_config.diameter_capstan_b / m_config.diameter_body_b,
      -t.z * m_config.diameter_capstan_c / m_config.diameter_body_c }; // switched sign 2016-05-30


  if(int(m_config.variant) == 1){ // ALUHAPTICS
      motorTorque[0] = motorTorque[0];
      motorTorque[1] = motorTorque[1];
      motorTorque[2] = -motorTorque[2];
  }
  // Set motor torque (t)
  double torque_constant[] = { m_config.torque_constant_motor_a,
                               m_config.torque_constant_motor_b,
                               m_config.torque_constant_motor_c };

  short signalToSend[3] = {0,0,0};
  int dir[3];
  int dir_chan[3] = {16,32,64}; // DIO4, DIO5, DIO6
  int dir_sum=0;

  for(int i=0;i<3;++i){
      double motorAmpere = motorTorque[i] / torque_constant[i];
      double signal = motorAmpere * 10.0 / m_config.current_for_10_v_signal;
      if(signal>10.0)  signal =  10.0;
      if(signal<-10.0) signal = -10.0;

//      dir[i] = cSign(signal) > 0 ? 1 : 2;
//      dir_sum += cSign(signal) < 0 ? dir_chan[i] : 0;
#if defined(USB) || defined(LAN)
      if(motorAmpere>3) motorAmpere = 3;
      if(motorAmpere<-3) motorAmpere = -3;
      signalToSend[i] = short(motorAmpere*1000);
#endif

#ifndef USB
#ifndef LAN
      // One at a time
      uint direction = cSign(v) > 0 ? 1 : 2;

      uint data[2];
      data[0] = dir_chan[i];
      data[1] = 0;
      S826_DioOutputWrite(0,data,direction);

      setVolt(signal,i);
#endif
#endif
  }

  Libremotehaptics::Vector3d lf;
  lf.x = signalToSend[0];
  lf.y = signalToSend[1];
  lf.z = signalToSend[2];
  rh.setForce(lf);




  current_values.torque = dv.torque;
  com_lock.unlock();
}

H3DUtil::PeriodicThread::CallbackCode
NiFalconHapticsDevice::com_func( void *data ) {
  NiFalconHapticsDevice *hd = 
    static_cast< NiFalconHapticsDevice * >( data );
  
  if( hd->getDeviceState() != HAPIHapticsDevice::ENABLED ) {
    return H3DUtil::PeriodicThread::CALLBACK_CONTINUE; }
  
  //hd->device->getFalconFirmware()->setHomingMode( true );
  //hd->device->runIOLoop();
  //if( hd->device->getFalconFirmware()->isHomed() ) {
  if(true){

    /*
    // get position
    std::array<double,3> pos = hd->device->getPosition();
    Vec3 position( pos[0] , pos[1] , (pos[2] - .150f) );
    
    // get buttons
    std::shared_ptr<FalconGrip> grip = hd->device->getFalconGrip();
    // It is safe to not check the number of digital outputs since
    // getDigitalInput in FalconGrip.h checks this for us.
    // Button input are grabbed this way in order to get them in the
    // same order as for FalconHapticsDevice (using novints api).
    HAPIInt32 button_status = // [ + N (logo) V ] -> [ logo V N + ]
      (grip->getDigitalInput(0)?0x08:0x00)|
      (grip->getDigitalInput(1)?0x04:0x00)|
      (grip->getDigitalInput(2)?0x01:0x00)|
      (grip->getDigitalInput(3)?0x02:0x00);

    Rotation orientation;

    // transfer values to/from haptics thread.
    hd->com_lock.lock();
    
    hd->current_values.position = position;
    hd->current_values.button_status = button_status;
    hd->current_values.orientation = orientation;

    std::array<double,3> f;
    f[0] = hd->current_values.force.x;
    f[1] = hd->current_values.force.y;
    f[2] = hd->current_values.force.z;

    hd->com_lock.unlock();

    // send force
    hd->device->setForce(f);
    */
 }
  
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

bool NiFalconHapticsDevice::setDeviceIndex( unsigned int i ) {
  if( getDeviceState() == HAPIHapticsDevice::UNINITIALIZED ) {
    index = i;
    return true;
  } else {
    return false;
  }
}

unsigned int NiFalconHapticsDevice::getNrConnectedFalconDevices() {
    /*
  libnifalcon::FalconDevice device;
#ifdef NIFALCONAPI_LIBUSB
  device.setFalconComm<libnifalcon::FalconCommLibUSB>();
#else
#ifdef NIFALCONAPI_LIBFTD2XX
  device.setFalconComm<libnifalcon::FalconCommFTD2XX>();
#else
#ifdef NIFALCONAPI_LIBFTDI
  device.setFalconComm<libnifalcon::FalconCommLibFTDI>();
#endif
#endif
#endif
  unsigned int num_falcons;
  if(!device.getDeviceCount(num_falcons)) {
    return 0;
  }
  return num_falcons;
  */
    return 1;
}

#endif
