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
/// \file NiFalconHapticsDevice.h
/// \brief Header file for NiFalconHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __NIFALCONHAPTICSDEVICE_H__
#define __NIFALCONHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_NIFALCONAPI
#define NIFALCONAPI_LIBUSB

#include "../../../libremotehaptics/libremotehaptics.h"











//------------------------------------------------------------------------------
// From CWoodenDevice.h
//------------------------------------------------------------------------------
// Our 12*4=48 byte message (used both up and down)
struct woodenhaptics_message {
    float position_x;
    float position_y;
    float position_z;
    float command_force_x;
    float command_force_y;
    float command_force_z;
    float actual_current_0;
    float actual_current_1;
    float actual_current_2;
    float temperature_0;
    float temperature_1;
    float temperature_2;

    woodenhaptics_message():position_x(0),position_y(0),position_z(0),
                            command_force_x(0),command_force_y(0),command_force_z(0),
                            actual_current_0(0),actual_current_1(0),actual_current_2(0),
                            temperature_0(0),temperature_1(0),temperature_2(0){}
};



struct hid_to_pc_message { // 9*2 = 18 bytes
    short encoder_a;
    short encoder_b;
    short encoder_c;
    short encoder_pen_a;
    short encoder_pen_b;
    short encoder_pen_c;
    unsigned short debug;
    unsigned short latency;
    unsigned short version;
};



struct pc_to_hid_message {  // 9*2 = 18 bytes
    short current_motor_a_mA;
    short current_motor_b_mA;
    short current_motor_c_mA;
    short force_motor_a_N;
    short force_motor_b_N;
    short force_motor_c_N;
    unsigned short debug;
    unsigned short latency;
    unsigned short version;
};
//------------------------------------------------------------------------------
































// forward declaration
namespace libnifalcon {
  class FalconDevice;
}

namespace HAPI {
 
  /// \ingroup HapticsDevices
  /// \class NiFalconHapticsDevice
  /// \brief Interface to the Falcon device through the open source library
  /// libAnyFalcon. Works on linux.
  class HAPI_API NiFalconHapticsDevice: public HAPIHapticsDevice {
  public:












      //------------------------------------------------------------------------------
      // From CWoodenDevice.h
      //------------------------------------------------------------------------------
      //! A collection of variables that can be set in ~/wooden_haptics.json
      struct configuration {
          double variant;                 // 0=WoodenHaptics default, 1=AluHaptics
          double diameter_capstan_a;      // m
          double diameter_capstan_b;      // m
          double diameter_capstan_c;      // m
          double length_body_a;           // m
          double length_body_b;           // m
          double length_body_c;           // m
          double diameter_body_a;         // m
          double diameter_body_b;         // m
          double diameter_body_c;         // m
          double workspace_origin_x;      // m
          double workspace_origin_y;      // m
          double workspace_origin_z;      // m
          double workspace_radius;        // m (for application information)
          double torque_constant_motor_a; // Nm/A
          double torque_constant_motor_b; // Nm/A
          double torque_constant_motor_c; // Nm/A
          double current_for_10_v_signal; // A
          double cpr_encoder_a;           // quadrupled counts per revolution
          double cpr_encoder_b;           // quadrupled counts per revolution
          double cpr_encoder_c;           // quadrupled counts per revolution
          double max_linear_force;        // N
          double max_linear_stiffness;    // N/m
          double max_linear_damping;      // N/(m/s)
          double mass_body_b;             // Kg
          double mass_body_c;             // Kg
          double length_cm_body_b;        // m     distance to center of mass
          double length_cm_body_c;        // m     from previous body
          double g_constant;              // m/s^2 usually 9.81 or 0 to
                                          //       disable gravity compensation

          // Set values
          configuration(const double* k):
            variant(k[0]),
            diameter_capstan_a(k[1]), diameter_capstan_b(k[2]), diameter_capstan_c(k[3]),
            length_body_a(k[4]), length_body_b(k[5]), length_body_c(k[6]),
            diameter_body_a(k[7]), diameter_body_b(k[8]), diameter_body_c(k[9]),
            workspace_origin_x(k[10]), workspace_origin_y(k[11]), workspace_origin_z(k[12]),
            workspace_radius(k[13]), torque_constant_motor_a(k[14]),
            torque_constant_motor_b(k[15]), torque_constant_motor_c(k[16]),
            current_for_10_v_signal(k[17]), cpr_encoder_a(k[18]), cpr_encoder_b(k[19]),
            cpr_encoder_c(k[20]), max_linear_force(k[21]), max_linear_stiffness(k[22]),
            max_linear_damping(k[23]), mass_body_b(k[24]), mass_body_c(k[25]),
            length_cm_body_b(k[26]), length_cm_body_c(k[27]), g_constant(k[28]){}

          configuration(){}
      };
      int lost_messages;
      //------------------------------------------------------------------------------











    /// Returns the current number of connected falcon devices.
    static unsigned int getNrConnectedFalconDevices();
    
    /// Constructor. device_index is the index of falcon device
    /// connected. Should not be larger than getNrConnectedFalconDevices() - 1.
    NiFalconHapticsDevice( unsigned int device_index = 0 );

    /// Destructor.
    virtual ~NiFalconHapticsDevice();
    /// Returns the index of the falcon device the instance of this class
    /// refers to.
    inline unsigned int getDeviceIndex() {
      return index;
    }

    /// Set the index of the falcon device the instance of this class
    /// refers to. This call is only valid before the device is initialized.
    /// If it is called after initialization it will do nothing.
    /// It returns true on success, and false otherwise.
    bool setDeviceIndex( unsigned int index );

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;

  protected:
    /// Implementation of updateDeviceValues using DHD API to get the values.
    virtual void updateDeviceValues( HAPIHapticsDevice::DeviceValues &dv,
                                     HAPITime dt );

    /// Implementation of sendOutput using DHD API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Initialize the haptics device. Use the HapticThread class in Threads.h
    /// as the thread for haptic rendering.
    /// \param _thread_frequency is the desired haptic frequency. 
    /// 1000 is the maximum allowed frequency that can be specified. Setting
    /// this parameter to -1 means run as fast as possible. It is recommended
    /// to use the default value for most users.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// Callback function for communication thread
    static H3DUtil::PeriodicThread::CallbackCode com_func( void *data );

    /// Callback handle to the com_func callback that is set up
    int com_func_cb_handle;

    /// Thread used to do communication with the haptics device
    H3DUtil::PeriodicThread *com_thread;

    /// Lock for exchanging data with the communication thread.
    H3DUtil::MutexLock com_lock;

    /// The current device values updated in the communicataion thread.
    /// Access to this structure must be contained within locking with 
    /// com_lock.
    DeviceValues current_values;

    /// The libnifalcon device class. Cannot use auto_ptr here because
    /// the class is forward declared.
    libnifalcon::FalconDevice * device;

    /// The index of the Falcon device that the instance of the class
    /// refers to.
    unsigned int index;



    //------------------------------------------------------------------------------
    // From CWoodenDevice.h
    //------------------------------------------------------------------------------
    Libremotehaptics rh;

    double totalTime;

    configuration m_config;
    woodenhaptics_message incoming_msg;
    woodenhaptics_message outgoing_msg;
    hid_to_pc_message hid_to_pc;
    pc_to_hid_message pc_to_hid;
    //------------------------------------------------------------------------------

  };
}

#endif //HAVE_NIFALCONAPI
#endif
